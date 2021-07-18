function collide = intersect_sat_lanelet(shape,lanelet)
% INTERSECT_SAT_LANELET         uses INTERSECT_SAT while testing only the bounds of the lanelet

    collide = false;
    
    nob = size(lanelet,1);
    
    % iterate over all right bound segements of lanelet
    for iter = 1:nob-1
        i = iter;
        % get vector according to bound segment
        edge_vector = [lanelet(i,LaneletInfo.rx)-lanelet(i+1,LaneletInfo.rx);lanelet(i,LaneletInfo.ry)-lanelet(i+1,LaneletInfo.ry)];
        % axis orthogonal to edge vector
        axis = [-edge_vector(2),edge_vector(1)];
        % normalize vector
        axis = axis/norm(axis);
        % calculate min and max dot product for interval
        [min1, max1] = project_polygon_interval(axis,shape);
        val2 = dot(axis,[lanelet(i,LaneletInfo.rx);lanelet(i,LaneletInfo.ry)]);
        % if the lanebound does not cut the polygon it is a separating axis
        if (min1 < val2 && max1 > val2)
            collide = true;
            return;
        end
        % now for the left bound
        edge_vector = [lanelet(i,LaneletInfo.lx)-lanelet(i+1,LaneletInfo.lx);lanelet(i,LaneletInfo.ly)-lanelet(i+1,LaneletInfo.ly)];
        axis = [-edge_vector(2),edge_vector(1)];
        axis = axis/norm(axis);
        [min1, max1] = project_polygon_interval(axis,shape);
        val2 = dot(axis,[lanelet(i,LaneletInfo.lx);lanelet(i,LaneletInfo.ly)]);
        if (min1 < val2 && max1 > val2)
            collide = true;
            return;
        end
    end
end

