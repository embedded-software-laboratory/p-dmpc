function collision = is_collision(shapes)

    n_shapes = length(shapes);
    
    collision = false;
    
    for i = 1 : n_shapes
        
        shape1 = shapes{i};
        
        for j = i : n_shapes
            
            if i == j
            
                continue;
            
            end
            
            shape2 = shapes{j};
            
            % area of overlapping shapes
            intersection = intersect(shape1, shape2);
            
            if intersection.NumRegions ~= 0   
                collision = true;
                return;
            end

        end
        
    end



end

