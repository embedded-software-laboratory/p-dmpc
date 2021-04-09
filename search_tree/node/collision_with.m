function collision = collision_with(index, shapes, displacement, midpoint, obstacles)
    collision = false;
    
    shape = polyshape(shapes{index}(1,:),shapes{index}(2,:),'Simplify',false);
    
    offset1 = 2.3;
    offset2 = 2.3;
    
    % how to solve the problem with obstacles? non-convex?
    % how are they stored?
    if(~isempty(obstacles))
        intersection = intersect(shape, obstacles);
        if intersection.NumRegions ~= 0   
            collision = true;
            return;
        end
    end
    
    for i = (index - 1) : -1 : 1
        % Area of overlapping shapes
        if collision_candidate(midpoint(:,index),midpoint(:,i),displacement(index),displacement(i),offset1,offset2)
            % check if polygons intersect
            if collide_sat(shapes{i},shapes{index})
                collision = true;
                return;
            end
        else
            continue;
        end
    end
end
