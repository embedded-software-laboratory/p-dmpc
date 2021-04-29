function collision = collision_with(index, shapes, scenario)
    collision = false;
    
    obstacles = scenario.obstacles;
            
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
        % check if polygons intersect
        if intersect_sat(shapes{i},shapes{index})
            collision = true;
            return;
        end
    end
end
