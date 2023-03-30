function collision = intersect_boundary(shapes_without_offset, boundary)
% INTERSECT_BOUNDARY     Wrapper for INTERSECT_SAT calls for a scenario boundary and check if there is intersection between the shape and the lanelets boundary.
% Returns true iff the shape intersect with the boundary

    collision = false;
    n_boundary = size(boundary,2); % number of boundary segments
    
    for i = 1:n_boundary
        n_point = length(boundary{1,i});% number of data points of the boundary segments
        
        for n = 1:n_point-1
            if intersect_sat(shapes_without_offset,boundary{1,i}(:,n:n+1))
                collision = true;
%                 disp('There is collision with the boundary')
                return;
            end
        end
%         disp('There is no collision with the boundary')

    end
end
