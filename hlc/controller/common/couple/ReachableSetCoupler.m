classdef ReachableSetCoupler < Coupler

    methods

        function [adjacency] = couple(~, options, ~, iter)
            % COUPLE Calculate coupling adjacency matrix
            %        Coupling check is performed by checking which
            %        reachable set overlap
            reachable_sets = iter.reachable_sets;

            reachable_sets_polyshape = cellfun(@(c) ...
                polyshape(c', Simplify = false), ...
                reachable_sets(:, end) ...
            );

            adjacency = zeros(options.amount, options.amount);

            % get bounding box of reachable sets in the last prediction horizon
            bound_boxes_x = zeros(options.amount, 2);
            bound_boxes_y = zeros(options.amount, 2);

            for iVeh = 1:options.amount
                [bound_boxes_x(iVeh, :), bound_boxes_y(iVeh, :)] = boundingbox(reachable_sets_polyshape(iVeh));
            end

            for veh_i = 1:(options.amount - 1)
                x_i = bound_boxes_x(veh_i, :);
                y_i = bound_boxes_y(veh_i, :);

                for veh_j = (veh_i + 1):options.amount
                    x_j = bound_boxes_x(veh_j, :);
                    y_j = bound_boxes_y(veh_j, :);
                    % use rectangles to approximate their reachable sets for a quick check
                    if x_i(1) >= x_j(2) || y_i(1) >= y_j(2) || x_i(2) <= x_j(1) || y_i(2) <= y_j(1)
                        % reachable sets are not overlapping
                        continue
                    end

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
