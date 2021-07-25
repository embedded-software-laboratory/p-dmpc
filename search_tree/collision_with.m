function collision = collision_with(index, shapes, scenario, iStep)
% COLLISION_WITH    Determine whether position has is a collision.

    collision = false;
    
    obstacles = scenario.obstacles;
            
    nobs = numel(obstacles);
    for i = 1:nobs
        if intersect_sat(shapes{index},obstacles{i}) 
            collision = true;
            return;
        end
    end
    
    if ~isempty(scenario.dynamic_obstacle_area)
        for i = 1:size(scenario.dynamic_obstacle_area,1)
            if intersect_sat(shapes{index},scenario.dynamic_obstacle_area{i,iStep}) 
                collision = true;
                return;
            end
        end
    end
    
    for i = (index - 1) : -1 : 1
        % check if polygons intersect
        if intersect_sat(shapes{i},shapes{index})
            collision = true;
            return;
        end
    end
    
    lane_idx = nonzeros(scenario.vehicle_to_lanelet(index,:))';
    for i = lane_idx
        if intersect_lanelets(shapes{index},scenario.lanelets{i})
            collision = true;
            return;
        end
    end
    
end
