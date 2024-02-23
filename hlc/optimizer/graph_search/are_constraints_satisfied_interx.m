function are_satisfied = are_constraints_satisfied_interx( ...
        iter, ...
        i_vehicle, ...
        shapes, ...
        shapes_for_boundary_check, ...
        i_step, ...
        vehicle_obstacles, ...
        lanelet_boundary, ...
        hdv_obstacles ...
    )
    % are_constraints_satisfied_interx    Determine whether position has a collision.
    assert(iter.amount == 1) % if not 1, code adaption is needed
    are_satisfied = true;
    % Note1: Shape must be closed!
    % Note2: The collision check order is important.
    % Normally, check collision with lanelet boundary last would be better.
    if InterX(shapes{i_vehicle}, vehicle_obstacles{i_step})
        % check collision with vehicle obstacles
        are_satisfied = false;
        return
    end

    is_hdv_obstacle = ~all(all(isnan(hdv_obstacles{i_step})));

    if (is_hdv_obstacle && ...
            InterX(shapes{i_vehicle}, hdv_obstacles{i_step}) ...
        )
        % check collision with manual vehicle obstacles
        are_satisfied = false;
        return
    end

    % check collision with lanelet obstacles
    if InterX(shapes_for_boundary_check{i_vehicle}, lanelet_boundary)
        are_satisfied = false;
        return
    end

end
