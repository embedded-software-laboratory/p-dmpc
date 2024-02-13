classdef PrioritizedExplorativeController < PrioritizedController

    properties (Access = private)
        iter_base
        info_base
        iter_array_tmp (1, :) cell
        info_array_tmp (1, :) cell
        solution_cost (:, 1) double

        computation_levels_of_vehicles
        base_computation_level
        n_computation_levels

        current_permutations

        belonging_vector_total

        should_keep_permutation (1, 1) logical
        permutations (1, :) cell
    end

    methods

        function obj = PrioritizedExplorativeController(options, plant, ros2_node)
            obj = obj@PrioritizedController(options, plant, ros2_node);
            obj.initialize_permutations(options.max_num_CLs);
        end

    end

    methods (Access = protected)

        function controller(obj)

            obj.prepare_iterations();

            for desired_computation_level = 1:obj.n_computation_levels

                % calculate priority information for permutation
                % where vehicle is on the desired_computation_level
                obj.prepare_permutation(desired_computation_level);

                obj.solve_permutation();
            end

            obj.compute_solution_cost();
            obj.send_solution_cost();
            obj.receive_solution_cost();
            obj.choose_solution();

            obj.remember_priorities();

        end

        function create_coupling_graph(obj)
            % call create_coupling_graph function of super class
            create_coupling_graph@PrioritizedController(obj)

            % store mapping: agents<->subgraph (for keeping priority permutation)
            obj.belonging_vector_total = conncomp(digraph(obj.iter.directed_coupling_sequential), 'Type', 'weak');

            % store mapping: agents<->topological level (for permuting priorities)
            obj.computation_levels_of_vehicles = Prioritizer.computation_levels_of_vehicles(obj.iter.directed_coupling_sequential);

            % topological level of controlled agent (computation level of agent in initial priority permutation)
            obj.base_computation_level = obj.computation_levels_of_vehicles(obj.plant.vehicle_indices_controlled);
        end

    end

    methods (Access = public)

        function result = get_n_computation_levels(obj)
            result = obj.n_computation_levels;
        end

        function prepare_iterations(obj)
            % initialize variable to store control results
            obj.info_base = ControlResultsInfo( ...
                1, ...
                obj.options.Hp ...
            );

            % set base iteration data
            obj.iter_base = obj.iter;

            % initialize
            obj.iter_array_tmp = {};
            obj.info_array_tmp = {};

            obj.n_computation_levels = max(obj.computation_levels_of_vehicles);
            obj.current_permutations = obj.computation_level_permutations();
        end

        function prepare_permutation(obj, desired_computation_level)

            obj.iter = obj.iter_base;
            obj.info = obj.info_base;

            % get index of permutation where the vehicle is on the current desired computation level
            i_permutation = find(obj.current_permutations(:, desired_computation_level) == obj.base_computation_level);

            % the ith permutation needs i-1 permutation steps
            obj.iter.priority_permutation = i_permutation;

            % replace every ocurrence of the old level index by the level index in the permuted priority list
            permuted_priority_list = obj.i11changem( ...
                obj.computation_levels_of_vehicles, ...
                1:obj.n_computation_levels, ...
                obj.current_permutations(i_permutation, :) ...
            );

            % check whether the assigned computation level matches the current permutation
            assert(permuted_priority_list(obj.plant.vehicle_indices_controlled) == desired_computation_level)

            [row_coupling, col_coupling] = find(obj.iter_base.directed_coupling);
            n_couplings = length(row_coupling);

            for i_coupling = 1:n_couplings
                veh_i = row_coupling(i_coupling);
                veh_j = col_coupling(i_coupling);

                % swap priority if assigned computation levels invert the coupling
                if permuted_priority_list(veh_i) > permuted_priority_list(veh_j)
                    % do not swap couplings that are ignored due to lanelet crossing area
                    % those can not be swapped in general since vehicles inside the crossing area
                    % can not consider the crossing area as obstacle
                    is_any_ignored = ~obj.iter_base.directed_coupling_reduced(veh_i, veh_j) ...
                        && ~obj.iter_base.directed_coupling_reduced(veh_j, veh_i);

                    if ~is_any_ignored
                        obj.swap_entries_all_coupling_matrices(veh_i, veh_j);
                    end

                end

            end

        end

        function solve_permutation(obj)

            obj.plan();

            % Send own data to other vehicles
            obj.publish_predictions();

            % temporarily store data
            obj.iter_array_tmp{obj.iter.priority_permutation} = obj.iter;
            obj.info_array_tmp{obj.iter.priority_permutation} = obj.info;
        end

        function compute_solution_cost(obj)

            n_solutions = length(obj.iter_array_tmp);
            obj.solution_cost = NaN(1, n_solutions);

            for i_solution = 1:n_solutions

                if ismember( ...
                        obj.plant.vehicle_indices_controlled, ...
                        obj.info_array_tmp{i_solution}.vehicles_fallback ...
                    )
                    % in case of fallback use maximum cost
                    cost_value = 1e9;
                else
                    % prefer solutions that are computed and have a low cost to come value in the last step
                    cost_value = obj.info_array_tmp{i_solution}.tree.get_cost( ...
                        obj.info_array_tmp{i_solution}.tree_path(end) ...
                    );
                end

                obj.solution_cost(i_solution) = cost_value;

            end

        end

        function send_solution_cost(obj)

            % broadcast info about solution
            obj.solution_cost_communication.send_message( ...
                obj.k, ...
                obj.solution_cost ...
            );
        end

        function receive_solution_cost(obj)
            % receive info about solutions
            sub_graph = obj.belonging_vector_total(obj.plant.vehicle_indices_controlled(1));
            sub_graph_vehicles = find(obj.belonging_vector_total == sub_graph);
            other_vehicles = setdiff(sub_graph_vehicles, obj.plant.vehicle_indices_controlled);

            for j_vehicle = other_vehicles
                % loop over vehicle from which the messages are read
                latest_msg_j = obj.solution_cost_communication.read_message( ...
                    j_vehicle, ...
                    obj.k, ...
                    throw_error = true ...
                );
                % calculate objective value
                obj.solution_cost = obj.solution_cost + latest_msg_j.solution_cost;
            end

        end

        function choose_solution(obj)
            [min_solution_cost, chosen_solution] = min(obj.solution_cost);
            chosen_solution = chosen_solution(1); % guarantee that it is a single integer

            % Keep permutation if it was feasible
            obj.should_keep_permutation = min_solution_cost < 1e8;

            obj.info = obj.info_array_tmp{chosen_solution};
            obj.iter = obj.iter_array_tmp{chosen_solution};
        end

        function remember_priorities(obj)
            priorities = Prioritizer.priorities_from_directed_coupling(obj.iter.directed_coupling);
            obj.prioritizer.current_priorities = priorities;
        end

    end

    methods (Access = private)

        function result = computation_level_permutations(obj)
            % start: prioritization from previous time step
            % always keep permutation
            permutation_indices = zeros(obj.n_computation_levels, 1);
            % get permutations for certain number of computation_levels
            permutations_n_levels = obj.permutations{obj.n_computation_levels};

            switch obj.n_computation_levels
                case 1
                    % only one permutation possible
                    permutation_indices = 1;
                    result = permutations_n_levels(permutation_indices, :);
                case 2
                    % only 1 combination of permutations possible
                    permutation_indices = 1:2;
                    result = permutations_n_levels(permutation_indices, :);
                otherwise
                    n_permutations = size(permutations_n_levels, 1);
                    permutation_rand_stream = RandStream("mt19937ar", Seed = obj.k);
                    % iteratively fill matrix of permutations
                    result = zeros(obj.n_computation_levels, obj.n_computation_levels);

                    if obj.should_keep_permutation
                        % use the previous best permutation (1..n_computation_levels)
                        % is ordered from priorities of last time step
                        permutation_indices(1) = n_permutations;
                    else
                        % use "opposite" ordering
                        permutation_indices(1) = 1;
                    end

                    % build matrix from chosen permutations
                    result(1, :) = permutations_n_levels(permutation_indices(1), :);
                    % start random choosing from the second index cause first one is fixed
                    start_index = 2;

                    % order the permutations randomly
                    rand_indices = randperm(permutation_rand_stream, n_permutations, n_permutations);

                    % this keeps track of the index of the next permutation to check
                    next_permutation = 1;

                    % start at second (start_index) permutation, since first one is fixed (eventually)
                    for nth_permutation = start_index:obj.n_computation_levels

                        while next_permutation <= length(rand_indices)
                            % get next permutation to check & shift index
                            permutation = permutations_n_levels(rand_indices(next_permutation), :);
                            next_permutation = next_permutation + 1;

                            % check whether the permutation is valid
                            % according to the already chosen permutations
                            % no no duplicate entries in any column/row
                            if obj.check_validity(result, permutation)
                                % save indices of used permutation to update rank afterwards
                                permutation_indices(nth_permutation) = rand_indices(next_permutation - 1);
                                break;
                            end

                        end

                        % build matrix from chosen permutations
                        result(nth_permutation, :) = permutations_n_levels(permutation_indices(nth_permutation), :);
                    end

            end

        end

        function swap_entries_all_coupling_matrices(obj, veh_i, veh_j)
            entry_1 = (veh_i - 1) * obj.options.amount + veh_j;
            entry_2 = (veh_j - 1) * obj.options.amount + veh_i;
            entries = [entry_1, entry_2];
            swapped_entries = fliplr(entries);
            obj.iter.weighted_coupling(entries) = obj.iter_base.weighted_coupling(swapped_entries);
            obj.iter.directed_coupling(entries) = obj.iter_base.directed_coupling(swapped_entries);
            obj.iter.directed_coupling_reduced(entries) = obj.iter_base.directed_coupling_reduced(swapped_entries);
            obj.iter.directed_coupling_sequential(entries) = obj.iter_base.directed_coupling_sequential(swapped_entries);
        end

        function initialize_permutations(obj, max_num_CLs)
            obj.permutations = cell(max_num_CLs, 1);

            for i_max_CL = 1:max_num_CLs
                obj.permutations{i_max_CL} = perms(1:i_max_CL);
            end

        end

        function is_valid = check_validity(~, permutations, permutation)
            is_valid = true;
            % remove empty (zero) rows (currently not filled)
            permutations(~any(permutations, 2), :) = [];
            % add permutation that needs to be checked
            permutations(end + 1, :) = permutation;

            [n_rows, n_cols] = size(permutations);

            for i_col = 1:n_cols
                % check whether column has a unique value in every row
                % rows dont need to be checked by definition of a permutation
                if length(unique(permutations(:, i_col))) ~= n_rows
                    is_valid = false;
                    break;
                end

            end

        end

        function mapout = i11changem(~, mapout, newcode, oldcode)
            % own changem function to circumvent necessarity of toolbox
            assert(numel(newcode) == numel(oldcode), 'newcode and oldecode must have the same number of elements');
            [toreplace, bywhat] = ismember(mapout, oldcode);
            mapout(toreplace) = newcode(bywhat(toreplace));
        end

    end

end
