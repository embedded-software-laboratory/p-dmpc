function [trim_inputs, trim_adjacency] = choose_trims(mpa_type, max_acceleration_per_dt, max_deceleration_per_dt)
    % CHOOSE_TRIMS  Choose trim set used.
    % NOTE: If new trim need to be added, please use successive number!
    arguments
        mpa_type (1, 1) MpaType = MpaType.single_speed;
        % max acceleration in LE / dt
        max_acceleration_per_dt double = 0.131
        max_deceleration_per_dt double = max_acceleration_per_dt
    end

    switch mpa_type
        case MpaType.single_speed
            %% 12 trims: two speeds (0, 0.5 and 0.6 m/s), 12 steering
            n_half = 5;
            steering_max = 0.6; % rad
            steering = linspace(-steering_max, steering_max, 2 * n_half + 1);

            v_step = 0.1;
            v_max = 0.8; % m/s
            v_profile = 0:v_step:v_max;

            speed_left = v_profile(end - n_half + 1:end);
            speed_right = fliplr(speed_left);
            speed = [speed_left v_max speed_right];
            ntrims = numel(steering) + 1;
            trim_inputs = [steering', speed'];
            trim_inputs = [zeros(1, 2); trim_inputs]; % add equilibrium trim
            trim_adjacency = ones(ntrims);
            % equilibrium is state 1
            % equilibrium is reachable from all states
            % and can reach all states, so all 1s is good
            % other states are connected by one hop
            trim_adjacency(2:end, 2:end) = trim_adjacency(2:end, 2:end) ...
                - triu(ones(ntrims - 1), 2) ...
                - tril(ones(ntrims - 1), -2);
        case MpaType.triple_speed
            %% 34 trims: three straight speeds (0.5, 0.75 and 1 m/s), 11 steering, one equilibrium state
            n_sixth = 5;
            steering_max = 0.6; %0.4135; % rad
            steering = linspace(-steering_max, steering_max, 2 * n_sixth + 1);

            v_step = 0.1;
            v_max_1 = 0.5; % m/s
            v_max_2 = 0.7; % m/s
            v_max_3 = 0.9; % m/s
            v_profile_1 = ones(1, 20) * v_max_1;
            v_profile_2 = ones(1, 20) * v_max_2;
            v_profile_3 = ones(1, 20) * v_max_3;

            speed_left_1 = v_profile_1(end - n_sixth + 1:end);
            speed_right_1 = fliplr(speed_left_1);
            speed_left_2 = v_profile_2(end - n_sixth + 1:end);
            speed_right_2 = fliplr(speed_left_2);
            speed_left_3 = v_profile_3(end - n_sixth + 1:end);
            speed_right_3 = fliplr(speed_left_3);
            speed = [speed_left_1 v_max_1 speed_right_1 speed_left_2 v_max_2 speed_right_2 speed_left_3 v_max_3 speed_right_3];
            ntrims = 3 * numel(steering) + 1;
            n_third = numel(steering);
            trim_inputs = [[steering'; steering'; steering'], speed'];
            trim_inputs = [zeros(1, 2); trim_inputs]; % add equilibrium trim
            trim_adjacency = ones(ntrims);
            % equilibrium is state 1
            % equilibrium is reachable from all states
            % and can reach all states, so all 1s is good
            % other states are connected by one hop
            trim_adjacency(2:end, 2:end) = trim_adjacency(2:end, 2:end) ...
                - triu(ones(ntrims - 1), 2) ...
                - tril(ones(ntrims - 1), -2);
            trim_adjacency(1, (n_third + 2):end) = 0;
            trim_adjacency((n_third + 2):end, 1) = 0;
            trim_adjacency((1 + n_third), (1 + n_third) + 1) = 0;
            trim_adjacency((1 + n_third) + 1, (1 + n_third)) = 0;
            trim_adjacency((1 + n_third * 2), (1 + n_third * 2) + 1) = 0;
            trim_adjacency((1 + n_third * 2) + 1, (1 + n_third * 2)) = 0;

            trims_same_steering = [2:n_third + 1, (n_third + 2):(2 * n_third + 1); (n_third + 2):(2 * n_third + 1), (2 * n_third + 2):ntrims];

            for entry = 1:size(trims_same_steering, 2)
                i = trims_same_steering(1, entry);
                j = trims_same_steering(2, entry);
                trim_adjacency(i, j) = 1;
                trim_adjacency(j, i) = 1;
            end

        case MpaType.realistic % acceleration constraint + increased steering at low speeds
            % resolution of MPA
            d_speed = min(max_acceleration_per_dt, max_deceleration_per_dt);
            % max d_speed between two adjacent nodes/trims depends on
            acc_max = 1.05 * max_acceleration_per_dt;
            dec_max = 1.05 * max_deceleration_per_dt;
            % assert stricly smaller due to inaccurencies in float
            assert(d_speed < acc_max);
            assert(d_speed < dec_max);
            speed_min = 0;
            speed_max = 0.8;
            % round speed_max
            speed_max = d_speed * round(speed_max / d_speed);
            speed_vec = (speed_min:d_speed:speed_max)';
            n_speeds = numel(speed_vec);

            d_steer = 0.5 * pi / 18;
            steer_max_lo_speed = d_steer * round((3 * pi / 18) / d_steer);
            steer_max_hi_speed = d_steer * round((2 * pi / 18) / d_steer);
            d_steer_max = 1.05 * d_steer;
            assert(d_steer < d_steer_max);
            steer_cla = cell(n_speeds, 1);

            steer_cla{1} = -steer_max_lo_speed:d_steer:steer_max_lo_speed;
            x = [speed_min + d_speed, speed_vec(3)];
            v = [steer_max_lo_speed, steer_max_hi_speed];
            % interpolate between hi and low steering angle
            for i_speed = 2:3
                xq = speed_vec(i_speed);
                max_steer = interp1(x, v, xq);
                max_steer = d_steer * round(max_steer / d_steer);
                min_steer = -max_steer;
                steer_cla{i_speed} = min_steer:d_steer:max_steer;
            end

            % apply low steering angle
            for i_speed = 4:n_speeds
                steer_cla{i_speed} = -steer_max_hi_speed:d_steer:steer_max_hi_speed;
            end

            [trim_inputs, trim_adjacency] = build_mpa( ...
                speed_vec ...
                , steer_cla ...
                , acc_max ...
                , dec_max ...
                , d_steer_max ...
            );

        otherwise
            error('unknown mpa trim type');
    end

end
