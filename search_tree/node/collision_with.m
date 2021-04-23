function collision = collision_with(index, shapes, displacement, midpoint, scenario)
    collision = false;
    
    obstacles = scenario.obstacles;
    
    % vehicle-dependent offset added to circular manuever approximation
    veh_offset = scenario.model.max_dist_cg+scenario.offset;
        
    nobs = numel(obstacles);
    if(nobs ~= 0)
        for i = 1:nobs
            if intersect_sat(shapes{index},obstacles{i}) 
                collision = true;
                return;
            end
        end
    end
    
    for i = (index - 1) : -1 : 1
        % Area of overlapping shapes
        if collision_candidate(midpoint(:,index),midpoint(:,i),displacement(index),displacement(i),veh_offset,veh_offset)
            % check if polygons intersect
            if intersect_sat(shapes{i},shapes{index})
                collision = true;
                return;
            end
        else
            continue;
        end
    end
end
