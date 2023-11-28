function are_satisfied = are_constraints_satisfied_sat( ...
        iter, ...
        iVeh, ...
        shapes, ...
        shapes_for_boundary_check, ...
        iStep, ...
        ~, ...
        ~, ...
        ~, ...
        ~, ...
        ~ ...
    )
    % are_constraints_satisfied_sat    Determine whether position has a collision.

    are_satisfied = true;

    obstacles = iter.obstacles;

    nobs = numel(obstacles);

    for i = 1:nobs

        if intersect_sat(shapes{iVeh}, obstacles{i})
            are_satisfied = false;
            return;
        end

    end

    if ~isempty(iter.dynamic_obstacle_area)

        for i = 1:size(iter.dynamic_obstacle_area, 1)

            if intersect_sat(shapes{iVeh}, iter.dynamic_obstacle_area{i, iStep})
                are_satisfied = false;
                return;
            end

        end

    end

    for i = (iVeh - 1):-1:1
        % check if polygons intersect
        if intersect_sat(shapes{i}, shapes{iVeh})
            are_satisfied = false;
            return;
        end

    end

    if ~isempty(iter.predicted_lanelet_boundary(iVeh, :))

        if intersect_lanelet_boundary(shapes_for_boundary_check{iVeh}, iter.predicted_lanelet_boundary(iVeh, :))
            are_satisfied = false;
            return;
        end

    end

    if ~any(iter.hdv_adjacency)

        for i = find(iter.hdv_adjacency)

            if intersect_sat(shapes{iVeh}, iter.hdv_reachable_sets{i, iStep})
                are_satisfied = false;
                return;
            end

        end

    end

end
