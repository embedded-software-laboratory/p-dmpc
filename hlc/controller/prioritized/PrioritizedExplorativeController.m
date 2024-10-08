classdef PrioritizedExplorativeController < PrioritizedController

    properties (Access = private)
        iter_base
        info_base
        iter_array_tmp (1, :) cell
        info_array_tmp (1, :) cell
        solution_cost (:, :) double % n_solutions x n_graphs

        computation_levels_of_vehicles
        base_computation_level
        n_computation_levels

        current_permutations

        belonging_vector_total
    end

    methods (Access = public)

        function obj = PrioritizedExplorativeController(options, plant, ros2_node)
            obj = obj@PrioritizedController(options, plant, ros2_node);
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

            % replace every occurrence of the old level index by the level index in the permuted priority list
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

                    obj.swap_entries_all_coupling_matrices(veh_i, veh_j);

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
            n_graphs = max(obj.belonging_vector_total);
            i_own_graph = obj.belonging_vector_total(obj.plant.vehicle_indices_controlled);
            obj.solution_cost = zeros(n_solutions, n_graphs);

            for i_solution = 1:n_solutions

                % get solution cost
                cost_value = obj.info_array_tmp{i_solution}.tree.get_cost( ...
                    obj.info_array_tmp{i_solution}.tree_path(end) ...
                );

                obj.solution_cost(i_solution, i_own_graph) = cost_value;

            end

        end

        function send_solution_cost(obj)

            % broadcast info about solution
            i_own_graph = obj.belonging_vector_total(obj.plant.vehicle_indices_controlled);
            obj.solution_cost_communication.send_message( ...
                obj.k, ...
                obj.solution_cost(:, i_own_graph) ...
            );
        end

        function receive_solution_cost(obj)
            % receive info about solutions

            for i_graph = 1:max(obj.belonging_vector_total)
                sub_graph_vehicles = find(obj.belonging_vector_total == i_graph);
                other_vehicles = setdiff(sub_graph_vehicles, obj.plant.vehicle_indices_controlled);

                for j_vehicle = other_vehicles
                    % loop over vehicle from which the messages are read
                    latest_msg_j = obj.solution_cost_communication.read_message( ...
                        j_vehicle, ...
                        obj.k, ...
                        throw_error = true ...
                    );
                    % calculate objective value
                    obj.solution_cost(:, i_graph) = obj.solution_cost(:, i_graph) + latest_msg_j.solution_cost;
                end

            end

        end

        function choose_solution(obj)
            unique_priorities = zeros(size(obj.belonging_vector_total));
            i_own_graph = obj.belonging_vector_total(obj.plant.vehicle_indices_controlled);

            for i_graph = 1:max(obj.belonging_vector_total)
                sub_graph_vehicles = obj.belonging_vector_total == i_graph;
                % Round solution cost to avoid numerical differences
                obj.solution_cost(:, i_graph) = round(obj.solution_cost(:, i_graph), 8);
                [~, chosen_solution] = min(obj.solution_cost(:, i_graph));

                if i_graph == i_own_graph
                    obj.info = obj.info_array_tmp{chosen_solution};
                    obj.iter = obj.iter_array_tmp{chosen_solution};
                    % communicate actual prediction with permutation index 0
                    % used by other vehicles to consider fallback while planning
                    obj.iter.priority_permutation = 0;
                    obj.publish_predictions();
                end

                sub_graph_directed_coupling = obj.iter_array_tmp{chosen_solution}.directed_coupling( ...
                    sub_graph_vehicles, ...
                    sub_graph_vehicles ...
                );
                priorities = Prioritizer.priorities_from_directed_coupling(sub_graph_directed_coupling);

                unique_priorities(sub_graph_vehicles) = priorities + max(unique_priorities);
            end

            obj.prioritizer.current_priorities = unique_priorities;

        end

        function result = get_n_computation_levels(obj)
            result = obj.n_computation_levels;
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

        end

        function create_coupling_graph(obj)
            % call create_coupling_graph function of super class
            create_coupling_graph@PrioritizedController(obj)

            % store mapping: agents<->subgraph (for keeping priority permutation)
            obj.belonging_vector_total = conncomp(digraph(obj.iter.directed_coupling_sequential), 'Type', 'weak');

            % store mapping: agents<->topological level (for permuting priorities)
            obj.computation_levels_of_vehicles = kahn(obj.iter.directed_coupling_sequential);

            % topological level of controlled agent (computation level of agent in initial priority permutation)
            obj.base_computation_level = obj.computation_levels_of_vehicles(obj.plant.vehicle_indices_controlled);
        end

        function handle_others_fallback(obj)

            for i_permutation = 1:obj.n_computation_levels
                obj.iter = obj.iter_array_tmp{i_permutation};
                obj.info = obj.info_array_tmp{i_permutation};

                handle_others_fallback@HighLevelController(obj);

                obj.iter_array_tmp{i_permutation} = obj.iter;
                obj.info_array_tmp{i_permutation} = obj.info;
            end

        end

        function store_control_info(obj)
            obj.compute_solution_cost();
            obj.send_solution_cost();
            obj.receive_solution_cost();
            obj.choose_solution();
            store_control_info@HighLevelController(obj);
        end

    end

    methods (Access = private)

        function result = computation_level_permutations(obj)
            % start: prioritization from previous time step
            % always keep permutation
            % iteratively fill matrix of permutations
            result = zeros(obj.n_computation_levels, obj.n_computation_levels);

            result(1, :) = 1:obj.n_computation_levels;

            permutation_rand_stream = RandStream("mt19937ar", Seed = obj.k);

            % first permutation is fixed, start
            nth_permutation = 2;

            % start with second row
            while nth_permutation <= obj.n_computation_levels
                permutation = zeros(1, obj.n_computation_levels);
                % each column in `is_level_allowed` represents possible values
                % for the level of the corresponding class.
                % Each class is an entry in the current row (=permutation) of `result`
                is_level_allowed = true(obj.n_computation_levels);
                % remove already picked levels in `result` from `is_level_allowed`
                for i_col = 1:size(result, 2)
                    levels_picked = nonzeros(result(:, i_col));
                    is_level_allowed(levels_picked, i_col) = false;
                end

                n_filled_cells = 0;
                is_permutation_valid = true;

                while n_filled_cells < obj.n_computation_levels
                    % find class with least possibilities for level
                    [n_possibilities, i_cell] = min(sum(is_level_allowed, 1));

                    % discard current permutation
                    if (n_possibilities == 0)
                        % the chosen random values led to a dead end
                        is_permutation_valid = false;
                        break;
                    end

                    % choose possibility at random
                    possibilities_for_cell = find(is_level_allowed(:, i_cell));

                    level_picked = possibilities_for_cell(randi( ...
                        permutation_rand_stream, ...
                        n_possibilities ...
                    ));
                    permutation(i_cell) = level_picked;

                    % remove possibility from other classes
                    is_level_allowed(level_picked, :) = false;
                    % set is_level_allowed for current class to true
                    % in order to avoid choosing it again
                    is_level_allowed(:, i_cell) = true;

                    n_filled_cells = n_filled_cells + 1;
                end

                if ~is_permutation_valid
                    continue;
                end

                % build matrix from chosen permutations
                result(nth_permutation, :) = permutation;
                nth_permutation = nth_permutation + 1;

            end

        end

        function swap_entries_all_coupling_matrices(obj, veh_i, veh_j)
            entry_1 = (veh_i - 1) * obj.options.amount + veh_j;
            entry_2 = (veh_j - 1) * obj.options.amount + veh_i;
            entries = [entry_1, entry_2];
            swapped_entries = fliplr(entries);
            obj.iter.weighted_coupling(entries) = obj.iter_base.weighted_coupling(swapped_entries);
            obj.iter.directed_coupling(entries) = obj.iter_base.directed_coupling(swapped_entries);
            obj.iter.directed_coupling_sequential(entries) = obj.iter_base.directed_coupling_sequential(swapped_entries);
        end

        function mapout = i11changem(~, mapout, newcode, oldcode)
            % own changem function to circumvent necessarity of toolbox
            assert(numel(newcode) == numel(oldcode), 'newcode and oldcode must have the same number of elements');
            [toreplace, bywhat] = ismember(mapout, oldcode);
            mapout(toreplace) = newcode(bywhat(toreplace));
        end

    end

end
