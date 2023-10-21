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

        % create polyshape of all predicted lanelets
        boundary_x = [ ...
                          predicted_lanelet_boundary{1}(1, :), ... % left
                          predicted_lanelet_boundary{2}(1, end:-1:1) ... % right
                      ];

        boundary_y = [ ...
                          predicted_lanelet_boundary{1}(2, :), ... % left
                          predicted_lanelet_boundary{2}(2, end:-1:1) ... % right
                      ];

        poly_boundary = polyshape(boundary_x, boundary_y, 'Simplify', false);

        for t = 1:Hp

            % compute intersection between reachable set and lanelet boundaries
            reachable_sets{t} = intersect(reachable_sets{t}, poly_boundary);

            if reachable_sets{t}.NumRegions > 1
                % remove unexpected small regions resulting from computing
                % the intersection

                % sort polyshape intersection by number of sides
                % separate each region of the polyshape into single polyshape
                % objects (a polyshape object could contain multiple regions)
                polyshape_regions = regions(sortregions( ...
                    reachable_sets{t}, ...
                    'numsides', ...
                    'descend' ...
                ));
                % take polyshape with the highest number of sides
                reachable_sets{t} = polyshape_regions(1);
            end

            if isempty(reachable_sets{t}.Vertices)
                % empty reachable set due to intersection with wrong
                % lanelet boundary

                % restore reachable set
                reachable_sets{t} = polyshape( ...
                    reachable_set_x, ...
                    reachable_set_y, ...
                    'Simplify', false ...
                );
            end

        end

    end

    % force convex reachable sets if non-convex polygons are not allowed
    if ~options.is_allow_non_convex
        reachable_sets = cellfun(@(c) convhull(c), reachable_sets, ...
            'UniformOutput', false ...
        );
    end

end
