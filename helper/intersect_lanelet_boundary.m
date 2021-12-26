function collision = intersect_lanelet_boundary(shapes_without_offset, boundary)
% INTERSECT_LANELET_BOUNDARY     check if there is intersection between the shape and the lanelets boundary.
% Returns true iff the shape intersect with the boundary

    collision = false;
    boundary_points = cell2mat(boundary);
    n_point = size(boundary_points,1);% number of data points of the boundary segments

    for n = 1:n_point-1
        %check leftBoundary
        if intersect_sat(shapes_without_offset,boundary_points(n:n+1,1:2)') || intersect_sat(shapes_without_offset,boundary_points(n:n+1,3:4)')
            
            collision = true;
%             disp('There is collision with the boundary')
            return;
        end
        
    end
%         disp('There is no collision with the boundary')

end
