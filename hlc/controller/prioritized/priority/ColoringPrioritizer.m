classdef ColoringPrioritizer < Prioritizer

    properties (Access = private)
    end

    methods

        function obj = ColoringPrioritizer()
        end

        function [directed_coupling] = prioritize(obj, iter, ~, ~, ~)
            adjacency = iter.adjacency;
            % apply topological sorting algorithm with coloring
            [topo_valid, topo_matrix] = ColoringPrioritizer.topological_coloring(adjacency);
            assert(topo_valid, 'No valid topological coloring possible.');

            % determine the order to reduce incoming edges
            order = ColoringPrioritizer.order_topo(topo_matrix, adjacency);

            % reorder matrix according to new level order
            topo_matrix(:, :) = topo_matrix(order, :);

            % Assign priority according to computation level
            directed_coupling = obj.direct_coupling(adjacency, topo_matrix);
        end

    end

    methods (Static, Access = private)

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
                v = ColoringPrioritizer.vertex_sdo_ldo(A, color, degree);
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

        function order = order_topo(L, C)
            % ORDER_TOPO  order topological levels in order to minimize their incoming edges
            %     ORDER = ORDER_TOPO(L,C) returns the order of the levels in L regarding the incoming edges in the adjacency C

            level = ColoringPrioritizer.computation_level_members(L);
            deg = sum(C);

            if sum(deg) == 0
                order = 1;
                return;
            end

            order = [];

            while sum(deg) ~= 0
                max_deg = 0;
                max_idx = 0;

                for i = 1:length(deg)

                    if deg(i) > max_deg
                        max_deg = deg(i);
                        max_idx = i;
                    end

                end

                lvl = 0;

                for j = 1:length(level)

                    if ismember(max_idx, level(j).members)
                        lvl = j;
                        break;
                    end

                end

                order = [order, lvl];
                deg(level(lvl).members) = 0;
            end

        end

        function level = computation_level_members(topological_grouping_matrix)
            % PB_PREDECESSOR_GROUPS   Returns the members and predecessors based on the topological levels.

            level = struct;

            for group_idx = 1:size(topological_grouping_matrix, 1)
                level(group_idx).members = find(topological_grouping_matrix(group_idx, :));

                if group_idx == 1
                    level(group_idx).predecessors = [];
                else
                    level(group_idx).predecessors = [level(group_idx - 1).predecessors level(group_idx - 1).members];
                end

            end

        end

    end

end
