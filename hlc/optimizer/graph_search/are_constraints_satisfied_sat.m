function are_satisfied = are_constraints_satisfied_sat( ...
        iter, ...
        i_vehicle, ...
        shapes, ...
        shapes_for_boundary_check, ...
        i_step, ...
        ~, ...
        ~, ...
        ~, ...
        ~, ...
        ~ ...
    )
    % are_constraints_satisfied_sat    Determine whether position has a collision.

    are_satisfied = true;

    for i = 1:numel(iter.obstacles)

        if intersect_sat(shapes{i_vehicle}, iter.obstacles{i})
            are_satisfied = false;
            return;
        end

    end

    if ~isempty(iter.dynamic_obstacle_area)

        for i = 1:size(iter.dynamic_obstacle_area, 1)

            if intersect_sat(shapes{i_vehicle}, iter.dynamic_obstacle_area{i, i_step})
                are_satisfied = false;
                return;
            end

        end

    end

    for i = (i_vehicle - 1):-1:1
        % check if polygons intersect
        if intersect_sat(shapes{i}, shapes{i_vehicle})
            are_satisfied = false;
            return;
        end

    end

    if ~isempty(iter.predicted_lanelet_boundary(i_vehicle, :))

        if intersect_lanelet_boundary(shapes_for_boundary_check{i_vehicle}, iter.predicted_lanelet_boundary(i_vehicle, :))
            are_satisfied = false;
            return;
        end

    end

    if ~any(iter.hdv_adjacency)

        for i = find(iter.hdv_adjacency)

            if intersect_sat(shapes{i_vehicle}, iter.hdv_reachable_sets{i, i_step})
                are_satisfied = false;
                return;
            end

        end

    end

end
