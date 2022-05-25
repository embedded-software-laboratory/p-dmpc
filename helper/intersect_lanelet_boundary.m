function collision = intersect_lanelet_boundary(shapes, boundary)
% INTERSECT_LANELET_BOUNDARY     check if there is intersection between the shape and the lanelets boundary.
% Returns true iff the shape intersect with the boundary

    collision = false;
    boundary_points_left = boundary{1};
    boundary_points_right = boundary{2};
    n_point_left = length(boundary_points_left);% number of data points of the boundary segments
    n_point_right = length(boundary_points_right);

    max_x = max(shapes(1,:));
    min_x = min(shapes(1,:));
    max_y = max(shapes(2,:));
    min_y = min(shapes(2,:));
%     
    for n = 1:n_point_left-1
        %check left boundary
        line_segment_left = boundary_points_left(:,n:n+1);
        if all(max_x < line_segment_left(1,:)) || all(min_x > line_segment_left(1,:)) || all(max_y < line_segment_left(2,:)) || all(min_y > line_segment_left(2,:))
            continue
        end
        if intersect_sat(shapes,boundary_points_left(:,n:n+1))     
            collision = true;
%             disp('There is collision with the boundary')
            return;
        end 
    end
    
    for i = 1:n_point_right-1
        %check right boundary
        line_segment_right = boundary_points_right(:,i:i+1);
        if all(max_x < line_segment_right(1,:)) || all(min_x > line_segment_right(1,:)) || all(max_y < line_segment_right(2,:)) || all(min_y > line_segment_right(2,:))
            continue
        end
        if intersect_sat(shapes,boundary_points_right(:,i:i+1))    
            collision = true;
%             disp('There is collision with the boundary')
            return;
        end
        
    end   

end
