function collision = collision_with(index, shapes)
    n_shapes = length(shapes);
    collision = false;
    for i = 1 : n_shapes
        if i == index || isempty(shapes{i})
            continue
        end
        
        % Area of overlapping shapes
        intersection = intersect(shapes{index}, shapes{i});
        if intersection.NumRegions ~= 0   
            collision = true;
            return;
        end
    end
end
