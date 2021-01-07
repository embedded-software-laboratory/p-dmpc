function collision = collision_with(index, shapes)
    collision = false;
    for i = (index - 1) : -1 : 1
        % Area of overlapping shapes
        intersection = intersect(shapes(index), shapes(i));
        if intersection.NumRegions ~= 0   
            collision = true;
            return;
        end
    end
end
