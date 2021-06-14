function collision = collision_with(index, shapes, scenario, iStep)
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
end
