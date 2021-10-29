function collision = intersect_boundary(shape, boundary)
% INTERSECT_BOUNDARY     Wrapper for INTERSECT_SAT calls for a scenario boundary and check if there is intersection between the shape and the boundary.

    collision = false;
    nboundary = size(boundary,2); % number of boundary segments
    for i = 1:nboundary
        npoint = length(boundary{1,nboundary});% number of data points of the boundary segments
        
        for n = 1:npoint-1
            if intersect_sat(shape,boundary{1,nboundary}(:,i:i+1))
                collision = true;
                disp('There is collision with the boundary')
                return;
            end
        end
        disp('There is no collision with the boundary')

    end
end
