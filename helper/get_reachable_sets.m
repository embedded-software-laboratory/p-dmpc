function reachable_sets = get_reachable_sets(x0, y0, yaw0, local_reachable_sets, predicted_lanelet_boundary, options)
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

    %     % take roughly 15% of computation time
    %     % Process optional input and Name-Value pair options
    %     [x0, y0, yaw0, local_reachable_sets, predicted_lanelet_boundary, is_allow_non_convex] = ...
    %         parse_inputs(x0, y0, yaw0, local_reachable_sets, varargin{:});

    Hp = size(local_reachable_sets, 2);
    reachable_sets = cell(1, Hp);

    % get the full reachable sets in global frame as polyshape
    for t = 1:Hp
        % translate the local reachable sets to global coordinates
        [reachable_set_x, reachable_set_y] = translate_global( ...
            yaw0, ...
            x0, ...
            y0, ...
            local_reachable_sets{t}.Vertices(:, 1)', ...
            local_reachable_sets{t}.Vertices(:, 2)' ...
        );

        reachable_sets{t} = polyshape( ...
            reachable_set_x, ...
            reachable_set_y, ...
            'Simplify', false ...
        );
    end

    % constrain the reachable sets by the boundaries of the predicted lanelets
    if ( ...
            options.scenario_type ~= ScenarioType.circle && ...
            options.bound_reachable_sets ...
        )

        [reachable_sets] = bound_reachable_sets( ...
            reachable_sets, ...
            predicted_lanelet_boundary{3} ...
        );

    end

    % force convex reachable sets if non-convex polygons are not allowed
    if ~options.is_allow_non_convex
        reachable_sets = cellfun(@(c) convhull(c), reachable_sets, ...
            'UniformOutput', false ...
        );
    end

end
