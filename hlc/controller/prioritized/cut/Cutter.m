classdef (Abstract) Cutter
    % CUTTER    Abstract class for parallelizing couplings

    methods (Abstract)
        min_cut(obj, M, must_not_in_same_subset, must_in_same_subset)
    end

    methods (Static, Access = public)

        function cutter = get_cutter(strategy)
            % GET_CUTTER  Given a cuttting strategy this function returns the corresponding cutter.

            switch strategy
                case CutStrategies.iterative_min_cut
                    cutter = IterativeMinCutter();
                case CutStrategies.milp_cut
                    cutter = MilpCutter();
            end

        end

    end

    methods (Access = public)

        function directed_coupling_sequential = cut(obj, M, coupling_info, max_num_CLs)
            directed_coupling_sequential = zeros(size(M));

            if max_num_CLs == 1
                % No sequential couplings
                return
            end

            % partition the given graph to subgraphs with a certain upper graph size.
            % The sum of weights of edges connecting subgraphs is the objective value and should be minimized.
            [~, subgraphs_info] = obj.graph_partitioning_algorithm(M, coupling_info, max_num_CLs);

            directed_coupling_sequential(M ~= 0) = 1;

            n_grps = length(subgraphs_info); % one subgraph corresponds to one parallel group

            for i_graph = 1:n_grps
                parallel_vertices = setdiff(1:length(M), subgraphs_info(i_graph).vertices);
                directed_coupling_sequential(subgraphs_info(i_graph).vertices, parallel_vertices) = 0;
                directed_coupling_sequential(parallel_vertices, subgraphs_info(i_graph).vertices) = 0;
            end

            directed_coupling_sequential = Cutter.sequentialize_edges( ...
                M, ...
                directed_coupling_sequential, ...
                max_num_CLs ...
            );

        end

    end

    methods (Access = protected)

        function [belonging_vector, subgraphs_info] = graph_partitioning_algorithm(obj, M, coupling_info, max_num_CLs)
            % GRAPH_PARTITIONING_ALGORITHM Partition the given edge-weighted
            % directed acyclic graph (DAG) M while ensuring
            % the size of each subgraph does not exceed the maximum depth max_num_CLs.
            %
            % INPUT:
            %   M: a square unsymmetric matrix that defines the target undirected graph to be partitioned.
            %   Can be either the adjacency matrix or the edge-weights matrix of the
            %   target graph. Edge-weights will not be considered if M is the dajacency
            %   matrix.
            %
            %   coupling_info: coupling information of each coupling pair
            %
            %   max_num_CLs: maximum depth (number of computation levels)
            %
            % OUTPUT:
            %   belonging_vector: a column vector whose values indicate which
            %   subgraphs the vertices belong to. For example,"belonging_vector =
            %   [1;2;2;1;3]" means the 1st subgraph = {1,4}, the 2nd subgraph = {2,3}
            %   and the 3rd subgraph = {5}.
            %
            %   subgraph_info: information of all the subgraphs, such as vertices, number
            %   of computation levels

            % Force parallely driving vehicles to be coupled sequentially
            multiple_vehs_drive_parallel = {};
            entries = find(M);
            amount = size(M, 1);

            if ~isempty(entries) && ~isempty([coupling_info{:}]) % in case is isnt a commonroad scenario or there are no couplings
                used_coupling_info = [coupling_info{entries}];
                % find all vehicles that drive in parallel
                parallel_pairs = find([used_coupling_info.is_move_side_by_side]);
                parallel_pairs_entries = entries(parallel_pairs);
                used_coupling_info_parallel = used_coupling_info(parallel_pairs);
                % sort according to the STAC so that parallel pair with higher STAC will be considered earlier
                [~, order] = sort([used_coupling_info_parallel.stac]);
                parallel_pairs_entries = parallel_pairs_entries(order);
                rows = mod(parallel_pairs_entries, amount);
                cols = ceil(parallel_pairs_entries / amount);
                vehs_drive_parallel = [rows, cols];
            else
                vehs_drive_parallel = [];
            end

            % Deal with multiple vehicles drive in parallel
            for iParl = 1:size(vehs_drive_parallel, 1)
                vehs_drive_parallel_i = vehs_drive_parallel(iParl, :);
                find_cell_idx = find(cellfun(@(c) any(ismember(vehs_drive_parallel_i, c)), multiple_vehs_drive_parallel));

                if isempty(find_cell_idx) && length(vehs_drive_parallel_i) <= max_num_CLs
                    % Add to a new parallel dirving group
                    multiple_vehs_drive_parallel{end + 1} = vehs_drive_parallel_i;
                else
                    new_members = sort(unique([multiple_vehs_drive_parallel{find_cell_idx}, vehs_drive_parallel_i]), 'ascend');
                    % Check if the allowed number computation levels is exceeded
                    if length(new_members) <= max_num_CLs
                        % Add to the existing parallel driving group
                        if length(find_cell_idx) == 1
                            multiple_vehs_drive_parallel{find_cell_idx} = new_members;
                        else
                            find_cell_idx = sort(find_cell_idx, 'ascend');
                            multiple_vehs_drive_parallel{find_cell_idx(1)} = new_members;
                            % delete others
                            to_delete = false(1, length(multiple_vehs_drive_parallel));
                            to_delete(find_cell_idx(2:end)) = true;
                            multiple_vehs_drive_parallel = multiple_vehs_drive_parallel(~to_delete);
                        end

                    end

                end

            end

            G_directed = digraph(M);

            assert(isdag(G_directed)) % check whether DAG

            % decompose the supergraph if it contains unconnected components
            [belonging_vector, ~] = conncomp(G_directed, 'Type', 'weak');

            %     plot_partitioned_graph(belonging_vector, M, 'ShowWeights', true)
            % number of graphs
            graph_indices = unique(belonging_vector);
            n_graphs = length(graph_indices);

            subgraphs_info(n_graphs) = struct('vertices', [], 'num_CLs', [], 'path_info', []);

            for i_graph = graph_indices
                vertices_i = find(belonging_vector == i_graph);

                % get all paths from source vertices to sinks, since the computation levels are only depends on those paths
                path_info_i = Cutter.get_longest_paths_from_sources_to_sinks(M(vertices_i, vertices_i));
                % store all paths from source vertices to sink vertices
                subgraphs_info(i_graph).vertices = vertices_i;
                subgraphs_info(i_graph).path_info = path_info_i;
                subgraphs_info(i_graph).num_CLs = path_info_i(1).length;
            end

            % get the current longest subgraph (subgraph who has the maximum number of computation levels)
            [num_CLs_longest_graph, longest_graph_ID] = max([subgraphs_info.num_CLs]);

            % cut the longest graph into two parts until no graph is longer than defined
            while num_CLs_longest_graph > max_num_CLs
                must_in_same_subset = {};
                vertices_longest_graph = find(belonging_vector == longest_graph_ID); % vertices that belong to the longest graph
                M_longest_graph = M(vertices_longest_graph, vertices_longest_graph);

                % do not separate vehicles driving in parallel
                find_parallel_vehs_group = find(cellfun(@(c) all(ismember(c, vertices_longest_graph)), multiple_vehs_drive_parallel));

                if ~isempty(find_parallel_vehs_group)
                    must_in_same_subset_global = multiple_vehs_drive_parallel(find_parallel_vehs_group);

                    for j = 1:length(must_in_same_subset_global)
                        [~, local_idx] = ismember(must_in_same_subset_global{j}, vertices_longest_graph);
                        must_in_same_subset{j} = local_idx;
                    end

                end

                % constraints on which paths must be cut
                %         must_not_in_same_subset = {subgraphs_info(longest_graph_ID).path_info(1).path}; % cell array
                must_not_in_same_subset = {};
                % call program to cut the longest graph to two parts
                belonging_vector_longest_graph = obj.min_cut(M_longest_graph, must_not_in_same_subset, must_in_same_subset);

                vertices_first_part = vertices_longest_graph((belonging_vector_longest_graph == 0));
                vertices_second_part = vertices_longest_graph((belonging_vector_longest_graph == 1));
                % part which has more vertices is considered as the first part
                if length(vertices_second_part) > length(vertices_first_part)
                    % swap two variables
                    [vertices_first_part, vertices_second_part] = swap(vertices_first_part, vertices_second_part);
                end

                % number of graphs add 1 after cutting the longest subgraph into two parts
                n_graphs = n_graphs + 1;

                % keep the index of the first part, assign the second part with the highest graph index
                belonging_vector(vertices_second_part) = n_graphs;

                path_info_first_part = Cutter.get_longest_paths_from_sources_to_sinks(M(vertices_first_part, vertices_first_part));
                % replace the previous longest graph with the first part
                subgraphs_info(longest_graph_ID).vertices = vertices_first_part;
                subgraphs_info(longest_graph_ID).path_info = path_info_first_part;
                subgraphs_info(longest_graph_ID).num_CLs = path_info_first_part(1).length;

                path_info_second_part = Cutter.get_longest_paths_from_sources_to_sinks(M(vertices_second_part, vertices_second_part));
                % add the second part to the end
                subgraphs_info(n_graphs).vertices = vertices_second_part;
                subgraphs_info(n_graphs).path_info = path_info_second_part;
                subgraphs_info(n_graphs).num_CLs = path_info_second_part(1).length;

                % get the current longest subgraph (subgraph who has the maximum number of computation levels)
                [num_CLs_longest_graph, longest_graph_ID] = max([subgraphs_info.num_CLs]);

                % visualize subgraphs
                %         plot_partitioned_graph(belonging_vector, M, 'ShowWeights', true)

            end

        end

    end

    methods (Static, Access = private)

        function directed_coupling_sequential = sequentialize_edges( ...
                directed_coupling_weighted, ...
                directed_coupling_sequential, ...
                max_num_CLs ...
            )
            % sequentialize_edges Sequentializes edges.
            % Since we cut vertices during the graph partitioning algorithm, there
            % might exist edges that can be sequential instead of parallel without
            % increasing the number of computation levels.
            % This function sequentializes those edges.

            directed_coupling_sequential_weighted = directed_coupling_sequential;
            directed_coupling_sequential_weighted(directed_coupling_sequential_weighted ~= 0) ...
                = directed_coupling_weighted(directed_coupling_sequential_weighted ~= 0);
            parallel_edges_weighted = directed_coupling_weighted - directed_coupling_sequential_weighted;
            [row, col, weights] = find(parallel_edges_weighted);

            % sort descending by coupling weights, we want to sequentialize
            % higher weights if possible
            [~, order_by_weight] = sort(weights, 'descend');
            row = row(order_by_weight);
            col = col(order_by_weight);

            levels_of_vehicles = Prioritizer.computation_levels_of_vehicles(directed_coupling_sequential);

            directed_graph = digraph(logical(directed_coupling_sequential));

            for i_parallel_edge = 1:length(row)

                vertex_starting = row(i_parallel_edge);
                vertex_ending = col(i_parallel_edge);
                level_starting = levels_of_vehicles(vertex_starting);
                level_ending = levels_of_vehicles(vertex_ending);

                % Option 1: check whether the ending vertex is in a lower
                % computation level
                if level_starting < level_ending
                    % edge can be sequentialized
                    directed_graph = directed_graph.addedge(vertex_starting, vertex_ending);
                    continue
                end

                % Option 2: check whether the ending vertex can be moved to a
                % level that is lower than the starting vertex
                vertices_ordered_after = directed_graph.bfsearch(vertex_ending);
                level_after_ending = max(levels_of_vehicles(vertices_ordered_after));
                added_levels = (level_starting - level_ending + 1);
                computation_levels_if_sequential = level_after_ending + added_levels;

                if computation_levels_if_sequential <= max_num_CLs
                    directed_graph = directed_graph.addedge(vertex_starting, vertex_ending);

                    levels_of_vehicles([vertex_ending; vertices_ordered_after]) = ...
                        levels_of_vehicles([vertex_ending; vertices_ordered_after]) ...
                        + added_levels;
                end

            end

            directed_coupling_sequential = directed_graph.adjacency;

        end

        function path_info = get_longest_paths_from_sources_to_sinks(M)
            % GET_LONGEST_PATHS_FROM_SOURCES_TO_SINKS Returns a struct contains the
            % all longest paths starting from source vertices to sink vertices.

            directed_adjacency = (M ~= 0);
            source_vertices = find(all(directed_adjacency == 0, 1)); % get all source vertices of the DAG
            sink_vertices = find(all(directed_adjacency == 0, 2))'; % get all sink vertices of the DAG
            n_sources = length(source_vertices);
            n_sinks = length(sink_vertices);

            path_info(n_sources * n_sinks) = struct('source', [], 'sink', [], 'path', [], 'length', []);

            count = 1;

            % Get the distances between source vertices and sink vertices.
            % NOTE that here purposely convert the edge-weights to be negative to
            % get the longest distance via MATLAB function `shortestpath`.
            G_negative_weighted = digraph(-directed_adjacency);

            for i = 1:n_sources
                source_i = source_vertices(i);

                for j = 1:n_sinks
                    sink_j = sink_vertices(j);
                    path_info(count).source = source_i;
                    path_info(count).sink = sink_j;
                    path_info(count).path = shortestpath(G_negative_weighted, source_i, sink_j);
                    path_info(count).length = length(path_info(count).path);
                    count = count + 1;
                end

            end

            % order the paths according to their lengths
            [~, descending_order] = sort([path_info.length], 'descend');
            path_info = path_info(descending_order);

        end

    end

end
