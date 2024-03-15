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

            arguments
                directed_coupling_weighted (:, :) double
                directed_coupling_sequential (:, :) logical
                max_num_CLs (1, 1) double
            end

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

            levels_of_vehicles = kahn(directed_coupling_sequential);

            for i_parallel_edge = 1:length(row)

                vertex_starting = row(i_parallel_edge);
                vertex_ending = col(i_parallel_edge);
                level_starting = levels_of_vehicles(vertex_starting);
                level_ending = levels_of_vehicles(vertex_ending);

                % Option 1: check whether the ending vertex is in a lower
                % computation level
                if level_starting < level_ending
                    % edge can be sequentialized
                    directed_coupling_sequential(vertex_starting, vertex_ending) = 1;
                    continue
                end

                % Option 2: check whether the ending vertex can be moved to a
                % level that is lower than the starting vertex
                directed_coupling_sequential_new = directed_coupling_sequential;
                directed_coupling_sequential_new(vertex_starting, vertex_ending) = 1;

                levels_of_vehicles_new = kahn(directed_coupling_sequential_new);

                if max(levels_of_vehicles_new) <= max_num_CLs
                    directed_coupling_sequential(vertex_starting, vertex_ending) = 1;
                    levels_of_vehicles = levels_of_vehicles_new;
                end

            end

        end

        function discovered_distances = search(directed_coupling, start_vertex)

            arguments (Input)
                directed_coupling (:, :) logical
                start_vertex (1, 1) double
            end

            arguments (Output)
                discovered_distances (1, :) double
            end

            discovered = false(1, size(directed_coupling, 2));
            discovered(start_vertex) = 1;

            i_level = 1;
            discovered_distances = -inf(size(discovered));
            discovered_distances(start_vertex) = 0;
            new_vertices_to_expand = discovered;

            while nnz(new_vertices_to_expand) ~= 0
                vertices_to_expand = find(new_vertices_to_expand);
                new_vertices_to_expand(:) = false;

                for vertex_to_expand = vertices_to_expand
                    discovered(vertex_to_expand) = 1;
                    neighbors = directed_coupling(vertex_to_expand, :);
                    new_vertices_to_expand = new_vertices_to_expand | (neighbors & ~discovered);
                end

                discovered_distances(new_vertices_to_expand) = i_level;
                i_level = i_level + 1;
            end

        end

    end

end
