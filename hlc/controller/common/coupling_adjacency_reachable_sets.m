function [adjacency] = coupling_adjacency_reachable_sets(reachable_sets)
    % COUPLING_ADJACENCY_REACHABLE_SETS Calculate coupling adjacency matrix
    %                                   Coupling check is performed by checking which
    %                                   reachable set overlap

    amount = size(reachable_sets, 1);

    adjacency = zeros(amount, amount);

    % get bounding box of reachable sets in the last prediction horizon
    bound_boxes_x = zeros(amount, 2); bound_boxes_y = zeros(amount, 2);

    for iVeh = 1:amount
        [bound_boxes_x(iVeh, :), bound_boxes_y(iVeh, :)] = boundingbox(reachable_sets{iVeh, end});
    end

    for veh_i = 1:(amount - 1)
        x_i = bound_boxes_x(veh_i, :);
        y_i = bound_boxes_y(veh_i, :);

        for veh_j = (veh_i + 1):amount
            % check whether two vehicles' reachable sets at the last prediction horizon overlap
            x_j = bound_boxes_x(veh_j, :);
            y_j = bound_boxes_y(veh_j, :);
            % use rectangles to approximate their reachable sets for a quick check
            if x_i(1) >= x_j(2) || y_i(1) >= y_j(2) || x_i(2) <= x_j(1) || y_i(2) <= y_j(1)
                % reachable sets are not overlapping
                continue
            end

            overlap_reachable_sets = intersect(reachable_sets{veh_i, end}, reachable_sets{veh_j, end});
            area_overlap = area(overlap_reachable_sets);

            if area_overlap > 1e-3 % a small threshold to tolerate measurement error of lenelet boundary
                adjacency(veh_i, veh_j) = 1;
                adjacency(veh_j, veh_i) = 1;
            end

        end

    end

end
