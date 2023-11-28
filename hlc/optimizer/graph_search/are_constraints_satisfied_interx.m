function are_satisfied = are_constraints_satisfied_interx( ...
        iter, ...
        iVeh, ...
        shapes, ...
        shapes_for_boundary_check, ...
        iStep, ...
        vehicle_obstacles, ...
        shapes_without_offset, ...
        lanelet_crossing_areas, ...
        lanelet_boundary, ...
        hdv_obstacles ...
    )
    % are_constraints_satisfied_interx    Determine whether position has a collision.
    assert(iter.amount == 1) % if not 1, code adaption is needed
    are_satisfied = true;
    % Note1: Shape must be closed!
    % Note2: The collision check order is important.
    % Normally, check collision with lanelet boundary last would be better.
    if InterX(shapes{iVeh}, vehicle_obstacles{iStep})
        % check collision with vehicle obstacles
        are_satisfied = false;
        return
    end

    if InterX(shapes_without_offset{iVeh}, lanelet_crossing_areas)
        % check collision with crossing area of lanelets
        are_satisfied = false;
        return
    end

    is_hdv_obstacle = ~all(all(isnan(hdv_obstacles{iStep})));

    if (is_hdv_obstacle && ...
            InterX(shapes{iVeh}, hdv_obstacles{iStep}) ...
        )
        % check collision with manual vehicle obstacles
        are_satisfied = false;
        return
    end

    % check collision with lanelet obstacles
    if InterX(shapes_for_boundary_check{iVeh}, lanelet_boundary)
        are_satisfied = false;
        return
    end

end
