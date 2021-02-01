function collision = collision_with(index, shapes, obstacles)
    collision = false;
    
    if(~isempty(obstacles))
        intersection = intersect(shapes(index), obstacles);
        if intersection.NumRegions ~= 0   
            collision = true;
            return;
        end
    end
    
    for i = (index - 1) : -1 : 1
        % Area of overlapping shapes
        intersection = intersect(shapes(index), shapes(i));
        if intersection.NumRegions ~= 0   
            collision = true;
            return;
        end
    end
end
