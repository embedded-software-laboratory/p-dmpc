function reachable_sets = get_reachable_sets(x0, y0, yaw0, local_reachable_sets, varargin)
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

    % Process optional input and Name-Value pair options
    [x0, y0, yaw0, local_reachable_sets, predicted_lanelet_boundary, is_allow_non_convex] = ...
        parse_inputs(x0, y0, yaw0, local_reachable_sets, varargin{:});

    Hp = size(local_reachable_sets, 2);
    reachable_sets = cell(1, Hp);

    % if vehicle has predicted the lanelet boundary, the reachable sets would be the intersection of the full reachable sets and the lanelet boundary 
    if ~isempty(predicted_lanelet_boundary)
        boundary_x = [predicted_lanelet_boundary{1}(1,:), predicted_lanelet_boundary{2}(1,end:-1:1)];
        boundary_y = [predicted_lanelet_boundary{1}(2,:), predicted_lanelet_boundary{2}(2,end:-1:1)];
        poly_boundary = polyshape(boundary_x, boundary_y, 'Simplify', false);
    end

    for t = 1:Hp
        % translate the offline calculated local reachable sets to global coordinate
        [reachable_set_x, reachable_set_y] = ...
            translate_global(yaw0, x0, y0, local_reachable_sets{t}.Vertices(:,1)', local_reachable_sets{t}.Vertices(:,2)');
        ploy_reachable_sets = polyshape(reachable_set_x, reachable_set_y, 'Simplify', false);

%         % MATLAB build-in function to rotate (around origin)
%         local_reachable_sets_rotated = rotate(local_reachable_sets{t},rad2deg(yaw0),[0,0]);
%         % MATLAB build-in function to translate 
%         ploy_reachable_sets = translate(local_reachable_sets_rotated,[x0,y0]);

        % constrain the reachable sets by lanelet boundary
        if ~isempty(predicted_lanelet_boundary)
            % The fourth cell in predicted_lanelet_boundary is the narrowed boundary while the third cell is the original boundary 
            % Without narrowing, two lanelets are considered as overlap if they only share the same boundary
            ploy_reachable_sets = intersect(ploy_reachable_sets, poly_boundary);
            
            if ploy_reachable_sets.NumRegions > 1
                % Remove unexpected small regions resulting from the intersection of the lanelet boundaries and reachable sets
                polysort = sortregions(ploy_reachable_sets,'numsides','descend'); % sort by number of sides
                ploy_reachable_sets = rmboundary(polysort,2:polysort.NumRegions);
            end       
        end

        if is_allow_non_convex
            reachable_sets{t} = ploy_reachable_sets;
        else
            % convexify reachable sets if non-convex polygons are not allowed
            reachable_sets{t} = convhull(ploy_reachable_sets); 
        end
    end

end


%% local function
function [x0, y0, yaw0, local_reachable_sets, predicted_lanelet_boundary, is_allow_non_convex] = ...
    parse_inputs(x0, y0, yaw0, local_reachable_sets, varargin)
    % Process optional input and Name-Value pair options

    default_predicted_lanelet_boundary = {}; % default to empty
    default_is_allow_non_convex = false; % default to not allow non-convex polygons

    p = inputParser;
    addRequired(p,'x0',@(x) isnumeric(x) && isscalar(x)); % must be numerical scalar
    addRequired(p,'y0',@(x) isnumeric(x) && isscalar(x)); % must be numerical scalar
    addRequired(p,'yaw0',@(x) isnumeric(x) && isscalar(x)); % must be numerical scalar
    addRequired(p,'local_reachable_sets',@(x) iscell(x)); % must be cell array
    addOptional(p,'predicted_lanelet_boundary', default_predicted_lanelet_boundary, @(x) iscell(x)); % cell array
    addOptional(p,'is_allow_non_convex', default_is_allow_non_convex, @(x) islogical(x)); % must be logical 
    parse(p, x0, y0, yaw0, local_reachable_sets, varargin{:}); % start parsing
    
    % get parsed inputs
    x0 = p.Results.x0;
    y0 = p.Results.y0;
    yaw0 = p.Results.yaw0;
    local_reachable_sets = p.Results.local_reachable_sets;
    predicted_lanelet_boundary = p.Results.predicted_lanelet_boundary;
    is_allow_non_convex = p.Results.is_allow_non_convex;

end