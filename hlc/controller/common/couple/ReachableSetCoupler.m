classdef ReachableSetCoupler < Coupler

    methods

        function [adjacency] = couple(~, options, ~, iter)
            % COUPLE Calculate coupling adjacency matrix
            %        Coupling check is performed by checking which
            %        reachable set overlap
            reachable_sets = iter.reachable_sets;

            reachable_sets_polyshape = cellfun(@(c) ...
                polyshape(c', Simplify = false, KeepCollinearPoints = true), ...
                reachable_sets(:, end) ...
            );

            adjacency = zeros(options.amount, options.amount);

            for veh_i = 1:(options.amount - 1)

                for veh_j = (veh_i + 1):options.amount

                    area_overlap = area(intersect( ...
                        reachable_sets_polyshape(veh_i), ...
                        reachable_sets_polyshape(veh_j) ...
                    ));

                    % small threshold to tolerate inaccuracies in lanelet boundaries
                    if area_overlap > 1e-3
                        adjacency(veh_i, veh_j) = 1;
                        adjacency(veh_j, veh_i) = 1;
                    end

                end

            end

        end

    end

end
