function reachable_sets = get_reachable_sets(x0, y0, yaw0, trim_current, local_reachable_sets, predicted_lanelet_boundary)
% GET_REACHABLE_SETS Calculate the reachable sets based on the current pose
% and trim of the vehicle by tranlating the local reachable sets. If
% vehicle's predicted lanelet boundary is available, the reachable sets
% will be constrained by the lanelet boundary. 
%
% Input: 
%   x0: x-coordinate in the global coordinate system
%   y0: y-coordinate in the global coordinate system
%   yaw0: yaw angle in the global coordinate system
%   trim_current: current trim
%   local_reachable_sets: local reachable sets without information about vehicle's pose
%   predicted_lanelet_boundary: the boundary of vehicle's predicted lanelets
% 
% Output:
%   reachable_sets: cell(1,Hp), the reachable sets in the defined predition horizon
% 
    Hp = size(local_reachable_sets, 2);
    reachable_sets = cell(1, Hp);

    % if vehicle has predicted the lanelet boundary, the reachable sets would be the intersection of the full reachable sets and the lanelet boundary 
    if ~isempty(predicted_lanelet_boundary)
        boundary_x = [predicted_lanelet_boundary{1}(:,1); predicted_lanelet_boundary{2}(end:-1:1,1)];
        boundary_y = [predicted_lanelet_boundary{1}(:,2); predicted_lanelet_boundary{2}(end:-1:1,2)];
        poly_boundary = polyshape(boundary_x, boundary_y, 'Simplify', true);
    end

    for t = 1:Hp
        % translate the offline calculated lacal reachable sets to global coordinate
        [reachable_set_x, reachable_set_y] = ...
            translate_global(yaw0, x0, y0, local_reachable_sets{trim_current,t}.Vertices(:,1)', local_reachable_sets{trim_current,t}.Vertices(:,2)');
        ploy_reachable_sets = polyshape(reachable_set_x, reachable_set_y);
        if ~isempty(predicted_lanelet_boundary)           
            reachable_sets_constrained = intersect(ploy_reachable_sets, poly_boundary);
            reachable_sets_constrained_conv = convhull(reachable_sets_constrained);
            reachable_sets{t}  = reachable_sets_constrained_conv; % convexify reachable sets since the axis separate theorem works only for convex polygon
%             plot(ploy_reachable_sets)
%             plot(poly_boundary)
%             plot(reachable_sets{t})
        else
            % else the reachable sets are the full reachable sets
            reachable_sets{t} = convhull(ploy_reachable_sets); % convexify reachable sets since the axis separate theorem works only for convex polygon
        end
    end

end