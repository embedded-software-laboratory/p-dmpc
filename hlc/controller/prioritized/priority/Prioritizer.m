classdef (Abstract) Prioritizer < handle
    % PRIORITIZER    Abstract class used for defining properties and methods used by priority based distributed controller.

    properties
        is_assign_unique_priority = false % whether to assign unique priority
    end

    methods (Abstract)
        prioritize(obj, iter, time_step, options, intersection_center)
    end

    methods (Static)

        function prioritizer = get_prioritizer(strategy)
            % GET_PRIORITIZER  Given a prioritization strategy this function returns the corresponding prioritizer.

            switch strategy
                case PriorityStrategies.coloring_priority
                    prioritizer = ColoringPrioritizer();
                case PriorityStrategies.constant_priority
                    prioritizer = ConstantPrioritizer();
                case PriorityStrategies.random_priority
                    prioritizer = RandomPrioritizer();
                case PriorityStrategies.FCA_priority
                    prioritizer = FcaPrioritizer();
                case PriorityStrategies.optimal_priority
                    prioritizer = ConstantPrioritizer();
                case PriorityStrategies.explorative_priority
                    prioritizer = ConstantPrioritizer();
            end

        end

    end

    methods (Static)

        function vehicle_to_computation_level = computation_levels_of_vehicles(directed_coupling)
            % COMPUTATION_LEVELS_OF_VEHICLES  Given the directed coupling matrix,
            % this function returns the computation level of each vehicle.
            L = kahn(directed_coupling);

            n_vehicles = size(L, 2);
            vehicle_to_computation_level = zeros(1, n_vehicles);

            for i_vehicle = 1:n_vehicles
                vehicle_to_computation_level(i_vehicle) = find(L(:, i_vehicle));
            end

        end

        function directed_coupling = direct_coupling(undirected_coupling, topo_groups)
            % determine directed adjacency
            directed_coupling = undirected_coupling;
            [rows, cols] = find(undirected_coupling ~= 0);

            for k = 1:length(rows)
                v_i = rows(k);
                v_j = cols(k);

                if v_i == v_j
                    directed_coupling(v_i, v_j) = 0;
                    continue
                end

                level_i = find(topo_groups(:, v_i) == 1);
                level_j = find(topo_groups(:, v_j) == 1);
                % edge comes from vertex in the front level, ends in vertex in
                % back level
                if level_i > level_j
                    directed_coupling(v_i, v_j) = 0;
                end

            end

        end

        function directed_coupling = directed_coupling_from_priorities(adjacency, current_priorities)
            % DIRECTED_COUPLING_FROM_PRIORITIES  Given the adjacency matrix and
            % the current priorities, this function returns the directed coupling matrix.

            directed_coupling = adjacency;
            n_agents = size(directed_coupling, 1);

            for i = 1:n_agents

                for j = 1:n_agents

                    if directed_coupling(i, j) && (current_priorities(i) > current_priorities(j))
                        directed_coupling(i, j) = 0;
                    end

                end

            end

        end

        function priorities = priorities_from_directed_coupling(directed_coupling)
            % PRIORITIES_FROM_DIRECTED_COUPLING  Given the directed coupling matrix,
            % this function returns the priorities.

            n_vehicles = size(directed_coupling, 1);
            priorities = zeros(1, n_vehicles);
            directed_coupling_digraph = digraph(directed_coupling);

            if isdag(directed_coupling_digraph)
                topological_order = toposort( ...
                    directed_coupling_digraph, ...
                    Order = 'stable' ...
                );
                priorities(topological_order) = 1:n_vehicles;
            end

        end

        function result = unique_priorities(adjacency)
            % UNIQUE_PRIORITIES  Given the adjacency matrix, this function returns
            % all the possible unique priority permutations. Unique in
            % this context means that no two priority permutations can result in
            % the same topologigal order.

            arguments (Input)
                adjacency (:, :) double;
            end

            arguments (Output)
                result (:, :) double; % n_vehicles x n_unique_priority_permutations
            end

            n_vehicles = size(adjacency, 1);
            result = zeros(n_vehicles, 0);
            directed_coupling_base = triu(adjacency, 1);
            [edge_row, edge_col] = find(directed_coupling_base);
            n_edges = length(edge_row);
            n_permutations = 2^n_edges;

            for i_permutation = 1:n_permutations
                flips = dec2bin(i_permutation - 1, n_edges) == '1';

                directed_coupling = directed_coupling_base;

                % Flip edges
                idx = sub2ind(size(directed_coupling), edge_row(flips), edge_col(flips));
                directed_coupling(idx) = 0;
                idx = sub2ind(size(directed_coupling), edge_col(flips), edge_row(flips));
                directed_coupling(idx) = 1;

                directed_coupling_digraph = digraph(directed_coupling);

                if isdag(directed_coupling_digraph)
                    topological_order = toposort(directed_coupling_digraph);
                    priority = zeros(1, n_vehicles);
                    priority(topological_order) = 1:n_vehicles;
                    result(:, end + 1) = priority; %#ok<AGROW>
                end

            end

        end

    end

end
