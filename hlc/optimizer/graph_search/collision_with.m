function collision = collision_with(iter, index, shapes, shapes_for_lanelet_check, iStep)
    % COLLISION_WITH    Determine whether position has is a collision.

    collision = false;

    obstacles = iter.obstacles;

    nobs = numel(obstacles);

    for i = 1:nobs

        if intersect_sat(shapes{index}, obstacles{i})
            collision = true;
            return;
        end

    end

    if ~isempty(iter.dynamic_obstacle_area)

        for i = 1:size(iter.dynamic_obstacle_area, 1)

            if intersect_sat(shapes{index}, iter.dynamic_obstacle_area{i, iStep})
                collision = true;
                return;
            end

        end

    end

    for i = (index - 1):-1:1
        % check if polygons intersect
        if intersect_sat(shapes{i}, shapes{index})
            collision = true;
            return;
        end

    end

    if ~isempty(iter.predicted_lanelet_boundary(index, :))

        if intersect_lanelet_boundary(shapes_for_lanelet_check{index}, iter.predicted_lanelet_boundary(index, :))
            collision = true;
            return;
        end

    end

    if ~any(iter.hdv_adjacency)

        for i = find(iter.hdv_adjacency)

            if intersect_sat(shapes{index}, iter.hdv_reachable_sets{i, iStep})
                collision = true;
                return;
            end

        end

    end

end
