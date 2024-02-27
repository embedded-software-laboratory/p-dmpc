classdef GreedyCutter < Cutter

    methods (Access = public)

        function directed_coupling_sequential = cut(~, M, max_num_CLs)
            directed_coupling_sequential = zeros(size(M));

            if max_num_CLs == 1
                % No sequential couplings
                return
            end

            directed_coupling_sequential = GreedyCutter.sequentialize_edges( ...
                M, ...
                directed_coupling_sequential, ...
                max_num_CLs ...
            );

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
                    
                    levels_of_vehicles(vertices_ordered_after) = ...
                         distances(directed_graph, vertex_ending, vertices_ordered_after) ...
                        + added_levels + 1;
                end

            end

            directed_coupling_sequential = full(directed_graph.adjacency);

        end

    end

end
