% INPUT shapes as Matrix [xs,ys]
% uses separating axis theorem -> only for convex polygons
function collide = collide_sat(shapes1,shapes2)
        
    collide = true;
    
    noe1 = length(shapes1(1,:));
    noe2 = length(shapes2(1,:));
    
    % iterate over all edges of polygon 1
    for iter = 1:noe1
        i = iter;
        ip1 = mod(iter,noe1)+1;
        % get vector according to polygon edge
        edge_vector = [shapes1(1,i)-shapes1(1,ip1);shapes1(2,i)-shapes1(2,ip1)];
        % axis orthogonal to edge vector
        axis = [-edge_vector(2),edge_vector(1)];
        % normalize vector
        axis = axis/norm(axis);
        % calculate min and max dot product for interval
        [min1, max1] = project_polygon_interval(axis,shapes1);
        [min2, max2] = project_polygon_interval(axis,shapes2);
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
        edge_vector = [shapes2(1,i)-shapes2(1,ip1);shapes2(2,i)-shapes2(2,ip1)];
        axis = [-edge_vector(2),edge_vector(1)];
        axis = axis/norm(axis);
        [min1, max1] = project_polygon_interval(axis,shapes1);
        [min2, max2] = project_polygon_interval(axis,shapes2);
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
