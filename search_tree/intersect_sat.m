function collide = intersect_sat(shape1,shape2)
% INTERSECT_SAT         intersection test for two CONVEX polygon
%   COLLIDE = INTERSECT_SAT(shape1,shape2) returns true iff polygons defined by the vector shapei = [xs;ys] overlap
%   The implementation uses the separating axis theorem and therefore can only be used for convex polygons.
%   Behaviour for using the function for non-convex polygons is undefined

    collide = true;
    
    noe1 = length(shape1(1,:));
    noe2 = length(shape2(1,:));
    
    % iterate over all edges of polygon 1
    for iter = 1:noe1
        i = iter;
        ip1 = mod(iter,noe1)+1;
        % get vector according to polygon edge
        edge_vector = [shape1(1,i)-shape1(1,ip1);shape1(2,i)-shape1(2,ip1)];
        % axis orthogonal to edge vector
        axis = [-edge_vector(2),edge_vector(1)];
        % normalize vector
        axis = axis/norm(axis);
        % calculate min and max dot product for interval
        [min1, max1] = project_polygon_interval(axis,shape1);
        [min2, max2] = project_polygon_interval(axis,shape2);
        % calculate distance between the two polygons
        if min1 < min2
            int_dist = min2 - max1;
        else
            int_dist = min1 - max2;
        end
        % if there is space between the two polygons then the polygons dont collide
        if int_dist > 0
            collide = false;
            return;
        end
    end
    
    % now the same for polygon 2
    for iter = 1:noe2
        i = iter;
        ip1 = mod(iter,noe2)+1;
        edge_vector = [shape2(1,i)-shape2(1,ip1);shape2(2,i)-shape2(2,ip1)];
        axis = [-edge_vector(2),edge_vector(1)];
        axis = axis/norm(axis);
        [min1, max1] = project_polygon_interval(axis,shape1);
        [min2, max2] = project_polygon_interval(axis,shape2);
        if min1 < min2
            int_dist = min2 - max1;
        else
            int_dist = min1 - max2;
        end
        if int_dist > 0
            collide = false;
            return;
        end
    end
end
