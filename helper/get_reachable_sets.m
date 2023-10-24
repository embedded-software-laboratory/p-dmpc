function reachable_sets = get_reachable_sets(x0, y0, yaw0, local_reachable_sets)
    % GET_REACHABLE_SETS Calculate the reachable sets based on the current pose
    % and trim of the vehicle by translating the local reachable sets.
    %
    % Output:
    %   the reachable sets in the defined prediction horizon
    %   reachable_sets (1, Hp) cell

    arguments
        x0 (1, 1) double % current x coordinate of the vehicle
        y0 (1, 1) double % current y coordinate of the vehicle
        yaw0 (1, 1) double % current yaw of the vehicle

        % local reachable sets of the vehicle's current trim
        local_reachable_sets (1, :) cell % (1, Hp)
    end

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

end
