classdef ColoringPrioritizer < Prioritizer

    properties (Access = private)
    end

    methods

        function obj = ColoringPrioritizer()
        end

        function [directed_coupling] = prioritize(obj, ~, ~, iter)
            adjacency = iter.adjacency;
            % apply topological sorting algorithm with coloring
            [topo_valid, topo_matrix] = topological_coloring(adjacency);
            assert(topo_valid, 'No valid topological coloring possible.');

            % determine the order to reduce incoming edges
            order = order_topo(topo_matrix, adjacency);

            % reorder matrix according to new level order
            topo_matrix(:, :) = topo_matrix(order, :);

            % Assign priority according to computation level
            directed_coupling = obj.direct_coupling(adjacency, topo_matrix);
        end

    end

    methods (Access = private)

        function [valid, L] = topological_coloring(A)
            % TOPOLOGICAL_COLORING  Find a topological sorting with a minimum number of levels (approximation)
            %                       based on graph coloring in an undirected graph.

            % init
            a = length(A(:, 1));
            col = 1:a;
            color = zeros(1, a);
            cum_matrix = cumsum(A);
            degree = cum_matrix(end, :);
            % assignment
            color(degree == 0) = 1;

            while ~all(color ~= 0)
                % get next vertex in coloring order
                v = vertex_sdo_ldo(A, color, degree);
                neighbor_col = unique(color(A(v, :) == 1));
                poss_col = setdiff(col, neighbor_col);
                color(v) = poss_col(1);
            end

            % topolical sorting matrix
            used_col = unique(color);
            k_col = length(used_col);
            L = zeros(k_col, a);

            for i = 1:k_col
                L(i, color == used_col(i)) = 1;
            end

            valid = isequal(sum(sum(L, 1)), a);
        end

        function idx = vertex_sdo_ldo(A, color, degree)
            % VERTEX_SDO_LDO    finde next vertex in color order with a combination of
            %                   "Saturation Degree Ordering" (SDO) and "Largest Degree Ordering" (LDO)

            max = -1;
            uncolored = find(color == 0);

            for i = uncolored
                d = length(nonzeros(unique(color(A(i, :) == 1))));

                if d > max
                    max = d;
                    idx = i;
                end

                if d == max

                    if degree(i) > degree(idx)
                        idx = i;
                    end

                end

            end

        end

    end

end
