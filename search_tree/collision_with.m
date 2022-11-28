function collision = collision_with(iter, index, shapes, shapes_for_lanelet_check, scenario, iStep)
% COLLISION_WITH    Determine whether position has is a collision.

    collision = false;
    
    obstacles = scenario.obstacles;
            
    nobs = numel(obstacles);
    for i = 1:nobs
        if intersect_sat(shapes{index},obstacles{i}) 
            collision = true;
            %disp(sprintf("vehicle id responsible for collision with obstacles: %d", scenario.vehicles.ID));
%             disp('there is collision with obstacles')
            return;
        end
    end
    
    if ~isempty(iter.dynamic_obstacle_area)
        for i = 1:size(iter.dynamic_obstacle_area,1)
            if intersect_sat(shapes{index}, iter.dynamic_obstacle_area{i,iStep:iStep+scenario.options.Hp-1}) 
                collision = true;
                %disp(sprintf("vehicle id responsible for collision with dynamic obstacles: %d", scenario.vehicles.ID));
%                 disp('there is collision with dynamic obstacles')
                return;
            end
        end
    end
    
    for i = (index - 1) : -1 : 1
        % check if polygons intersect
        if intersect_sat(shapes{i},shapes{index})
            %disp(sprintf("vehicle id responsible for collision with vehicles: %d", scenario.vehicles.ID));
            collision = true;
%             disp('there is collision between two vehicles')
            return;
        end
    end
    
    if ~isempty(scenario.vehicle_to_lanelet)
        lane_idx = nonzeros(scenario.vehicle_to_lanelet(index,:))';
        for i = lane_idx
            %if intersect_lanelets(shapes{index},scenario.lanelet_boundary{i})
            if intersect_lanelets(shapes{index}, scenario.lanelets{i})
                %disp(sprintf("vehicle id responsible for collision with lanelets: %d", scenario.vehicles.ID));
                collision = true;
                return;
            end
        end
    end
    

    if ~isempty(scenario.vehicles(1,index).lanelet_boundary) % TODO use iter.lanelet_boundary
        if intersect_lanelet_boundary(shapes_for_lanelet_check{index}, scenario.vehicles(1,index).lanelet_boundary) % TODO use iter.lanelet_boundary
            %disp(sprintf("vehicle id responsible for collision: %d", scenario.vehicles.vehicle_id));
            collision = true;
            return;
        end
    end

    % check if collides with the reachable sets of coupling vehicles with higher priorities 
    if ~isempty(iter.dynamic_obstacle_reachableSets)
        for i = 1:size(iter.dynamic_obstacle_reachableSets,1)
            if intersect_sat(shapes{index},iter.dynamic_obstacle_reachableSets{i,iStep}) 
                collision = true;
                return;
            end
        end
    end
 
end
