classdef ReachableSetCoupler < Coupler

    methods

        function [adjacency] = couple(~, options, ~, iter)
            % COUPLE Calculate coupling adjacency matrix
            %        Coupling check is performed by checking which
            %        reachable set overlap
            reachable_sets = iter.reachable_sets;

            adjacency = zeros(options.amount, options.amount);

            for veh_i = 1:(options.amount - 1)
                x_i = reachable_sets{veh_i, end}(1, :);
                y_i = reachable_sets{veh_i, end}(2, :);

                for veh_j = (veh_i + 1):options.amount

                    if InterX(reachable_sets{veh_i, end}, reachable_sets{veh_j, end})
                        adjacency(veh_i, veh_j) = 1;
                        adjacency(veh_j, veh_i) = 1;
                    else
                        % check if two points are equal -- colinear lines are
                        % missed by InterX
                        x_j = reachable_sets{veh_j, end}(1, :);
                        y_j = reachable_sets{veh_j, end}(2, :);

                        [x_mat_i, x_mat_j] = meshgrid(x_i, x_j);
                        [y_mat_i, y_mat_j] = meshgrid(y_i, y_j);

                        if any(x_mat_i == x_mat_j & y_mat_i == y_mat_j, 'all')
                            adjacency(veh_i, veh_j) = 1;
                            adjacency(veh_j, veh_i) = 1;
                        end

                    end

                end

            end

        end

    end

end
