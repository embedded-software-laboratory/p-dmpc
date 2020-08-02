function collision = is_collision(shapes)

    nShapes = length(shapes);
    
    collision = false;
    
    for i = 1 : nShapes
        
        shape1 = shapes{i};
        
        for j = i : number_of_vehicles
            
            shape2 = shapes{j};
            
            % area of overlapping shapes
            intersection = boundary(intersect(shape1, shape2));
            
            if i ~= j && ~isempty(intersection)     
                collision = true;
                return;
            end

        end
        
    end



end

