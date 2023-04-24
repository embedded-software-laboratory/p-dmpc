function [trim_inputs, trim_adjacency] = build_mpa( ...
        speed_vec ...
        , steer_cla ...
        , acc_max ...
        , dec_max ...
        , d_steer_max ...
    )

    arguments
        speed_vec
        steer_cla
        acc_max
        dec_max
        d_steer_max
    end

    % steer_cla steering angles for corresponding speed, cell array of size (n_speeds x 1) [rad]
    assert(isequal(size(speed_vec), size(steer_cla)));

    n_speeds = numel(speed_vec);
    steer_all = [steer_cla{:}];
    n_steers = numel(steer_all);

    % STATES
    % n_steers contains all possible steers for each speed
    n_trims = n_steers;
    trim_inputs = zeros(n_trims, 2);
    n_steer_of_speed = zeros(n_speeds, 1);

    for i_speed = 1:n_speeds
        % number of different steerings available at speed i_speed
        n_steer_of_speed(i_speed) = numel(steer_cla{i_speed});

        for i_steer = 1:n_steer_of_speed(i_speed)
            c_steer = steer_cla{i_speed}(i_steer);
            c_speed = speed_vec(i_speed);
            i_min_trim = sum(n_steer_of_speed(1:i_speed - 1));
            i_trim = i_min_trim + i_steer;
            trim_inputs(i_trim, :) = [c_steer, c_speed];
        end

    end

    % TRANSITIONS
    trim_adjacency = zeros(n_trims);

    for i = 1:n_trims

        for j = 1:n_trims
            steering_condition = (abs(trim_inputs(j, 1) - trim_inputs(i, 1)) <= d_steer_max);

            if steering_condition == true

                if trim_inputs(j, 2) > trim_inputs(i, 2)
                    is_transition = (trim_inputs(j, 2) - trim_inputs(i, 2)) <= acc_max;
                else
                    is_transition = (trim_inputs(i, 2) - trim_inputs(j, 2)) <= dec_max;
                end

            else
                is_transition = false;
            end

            if is_transition
                trim_adjacency(i, j) = 1;
            end

        end

    end

end
