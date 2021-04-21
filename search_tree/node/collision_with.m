function collision = collision_with(index, shapes, displacement, midpoint, obstacles, offset)
    collision = false;
        
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
        if collision_candidate(midpoint(:,index),midpoint(:,i),displacement(index),displacement(i),offset,offset)
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
