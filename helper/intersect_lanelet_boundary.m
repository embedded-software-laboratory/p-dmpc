function collision = intersect_lanelet_boundary(shapes, boundary)
% INTERSECT_LANELET_BOUNDARY     check if there is intersection between the shape and the lanelets boundary.
% Returns true iff the shape intersect with the boundary

    collision = false;
    boundary_points_left = boundary{1,1}';
    boundary_points_right = boundary{1,2}';
    n_point_left = length(boundary_points_left);% number of data points of the boundary segments
    n_point_right = length(boundary_points_right);
    
    for n = 1:n_point_left-1
        %check left boundary
        if intersect_sat(shapes,boundary_points_left(:,n:n+1))     
            collision = true;
%             disp('There is collision with the boundary')
            return;
        end
        
    end
    
    for i = 1:n_point_right-1
        %check right boundary
        if intersect_sat(shapes,boundary_points_right(:,i:i+1))    
            collision = true;
%             disp('There is collision with the boundary')
            return;
        end
        
    end    
    

end
