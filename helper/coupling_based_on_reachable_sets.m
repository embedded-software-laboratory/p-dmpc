function [ scenario ] = coupling_based_on_reachable_sets(scenario, iter)
    
    adjacency = zeros(scenario.options.amount,scenario.options.amount);

    % get bounding box of reachable sets in the last prediction horizon
    bound_boxes_x = zeros(scenario.options.amount,2); bound_boxes_y = zeros(scenario.options.amount,2);
    for iVeh = 1:scenario.options.amount
        [bound_boxes_x(iVeh,:),bound_boxes_y(iVeh,:)] = boundingbox(iter.reachable_sets{iVeh,end});
    end

    for veh_i = 1:(scenario.options.amount-1)
        x_i = bound_boxes_x(veh_i,:);
        y_i = bound_boxes_y(veh_i,:);
        
        for veh_j = (veh_i+1):scenario.options.amount
            % check whether two vehicles' reachable sets at the last prediction horizon overlap
            x_j = bound_boxes_x(veh_j,:);
            y_j = bound_boxes_y(veh_j,:);
            % use rectangles to approximate their reachable sets for a quick check
            if x_i(1)>=x_j(2) || y_i(1)>=y_j(2) || x_i(2)<=x_j(1) || y_i(2)<=y_j(1)
                % reachable sets are not overlapping
                continue
            end
            overlap_reachable_sets = intersect(iter.reachable_sets{veh_i,end}, iter.reachable_sets{veh_j,end});
            area_overlap = area(overlap_reachable_sets);
            if area_overlap > 1e-3 % a small threshold to tolerate measurement error of lenelet boundary
                adjacency(veh_i,veh_j) = 1;
                adjacency(veh_j,veh_i) = 1;
            end  
        end
    end

    scenario.adjacency(:,:,scenario.k) = adjacency;
    %scenario.semi_adjacency(:,:,scenario.k) = adjacency;
end

