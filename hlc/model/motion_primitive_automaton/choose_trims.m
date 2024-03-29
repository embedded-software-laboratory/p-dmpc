function [trim_inputs, trim_adjacency] = choose_trims(trim_set, max_acceleration_per_dt, max_deceleration_per_dt)
    % CHOOSE_TRIMS  Choose trim set used.
    % NOTE: If new trim need to be added, please use successive number!
    arguments
        trim_set (1, 1) int8 = 14
        % max acceleration in LE / dt
        max_acceleration_per_dt double = 0.131
        max_deceleration_per_dt double = max_acceleration_per_dt
    end

    switch trim_set
        case 0 % trim set for manual vehicles TODO: graph weight must be nonnegative
            %% 6 trims: two speeds (-1.5 m/s and 1.5 m/s), three steering (-pi/9, 0, pi/9)
            steering = (-1:1) * pi / 9;
            ntrims = numel(steering) * 2;
            trim_inputs = zeros(ntrims, 2);
            trim_inputs(1:ntrims / 2, 1) = steering;
            trim_inputs(ntrims / 2 + 1:end, 1) = steering;
            trim_inputs(1:ntrims / 2, 2) = -1.5;
            trim_inputs(ntrims / 2 + 1:end, 2) = 1.5;
            % All states connected
            trim_adjacency = ones(ntrims);
        case 1
            %% 3 trims: three speeds (0, 0.75 and 1.5 m/s), one steering (0 deg)
            speeds = 0:0.75:1.5;
            ntrims = numel(speeds);
            trim_inputs = zeros(ntrims, 2);
            trim_inputs(:, 2) = speeds;
            trim_adjacency = eye(ntrims);
            trim_adjacency(1:end - 1, 2:end) = trim_adjacency(1:end - 1, 2:end) ...
                + eye(ntrims - 1);
            trim_adjacency(2:end, 1:end - 1) = trim_adjacency(2:end, 1:end - 1) ...
                + eye(ntrims - 1);

        case 2
            % In manual mode: low speed profile
            %% 12 trims: two speeds (0, 0.05 and 0.6 m/s), 12 steering
            steering = (-2:0.5:2) * pi / 18;
            ntrims = numel(steering) + 3;
            trim_inputs = zeros(ntrims, 2);
            trim_inputs(2, 1) = -0.6;
            trim_inputs(2, 2) = 0.05;
            trim_inputs(end, 1) = 0.6;
            trim_inputs(end, 2) = 0.05;
            trim_inputs(3:end - 1, 1) = steering;
            trim_inputs(3:end - 1, 2) = 0.6;
            trim_adjacency = ones(ntrims);
            % equilibrium is state 1
            % equilibrium is reachable from all states
            % and can reach all states, so all 1s is good
            % other states are connected by one hop
            trim_adjacency(2:end, 2:end) = trim_adjacency(2:end, 2:end) ...
                - triu(ones(ntrims - 1), 2) ...
                - tril(ones(ntrims - 1), -2);

        case 3
            %% 4 trims: two speeds (0 and 0.75 m/s), three steering (-pi/9 and pi/9)
            steering = (-1:1) * pi / 9;
            ntrims = numel(steering) + 1;
            trim_inputs = zeros(ntrims, 2);
            trim_inputs(2:end, 1) = steering;
            trim_inputs(2:end, 2) = 0.75; %10 / 3.6;
            % All states connected
            trim_adjacency = ones(ntrims);

        case 4
            % In manual mode: medium speed profile
            %% 12 trims: two speeds (0, 0.05 and 0.8 m/s), 12 steering
            steering = (-2:0.5:2) * pi / 18;
            ntrims = numel(steering) + 3;
            trim_inputs = zeros(ntrims, 2);
            trim_inputs(2, 1) = -0.6;
            trim_inputs(2, 2) = 0.05;
            trim_inputs(end, 1) = 0.6;
            trim_inputs(end, 2) = 0.05;
            trim_inputs(3:end - 1, 1) = steering;
            trim_inputs(3:end - 1, 2) = 0.8;
            trim_adjacency = ones(ntrims);
            % equilibrium is state 1
            % equilibrium is reachable from all states
            % and can reach all states, so all 1s is good
            % other states are connected by one hop
            trim_adjacency(2:end, 2:end) = trim_adjacency(2:end, 2:end) ...
                - triu(ones(ntrims - 1), 2) ...
                - tril(ones(ntrims - 1), -2);

        case 5
            %% 6 trims: two speeds (0 and 0.75 m/s), five steering
            steering = (-2:2) * pi / 18;
            ntrims = numel(steering) + 1;
            trim_inputs = zeros(ntrims, 2);
            trim_inputs(2:end, 1) = steering;
            trim_inputs(2:end, 2) = 0.75; %10 / 3.6;
            trim_adjacency = ones(ntrims);
            % equilibrium is state 1
            % equilibrium is reachable from all states
            % and can reach all states, so all 1s is good
            % other states are connected by one hop
            trim_adjacency(2:end, 2:end) = trim_adjacency(2:end, 2:end) ...
                - triu(ones(ntrims - 1), 2) ...
                - tril(ones(ntrims - 1), -2);

        case 6
            % In manual mode: high speed profile
            %% 12 trims: two speeds (0, 0.5 and 1.2 m/s), 12 steering
            steering = (-2:0.5:2) * pi / 18;
            ntrims = numel(steering) + 3;
            trim_inputs = zeros(ntrims, 2);
            trim_inputs(2, 1) = -0.6;
            trim_inputs(2, 2) = 0.05;
            trim_inputs(end, 1) = 0.6;
            trim_inputs(end, 2) = 0.05;
            trim_inputs(3:end - 1, 1) = steering;
            trim_inputs(3:end - 1, 2) = 1.2;
            trim_adjacency = ones(ntrims);
            % equilibrium is state 1
            % equilibrium is reachable from all states
            % and can reach all states, so all 1s is good
            % other states are connected by one hop
            trim_adjacency(2:end, 2:end) = trim_adjacency(2:end, 2:end) ...
                - triu(ones(ntrims - 1), 2) ...
                - tril(ones(ntrims - 1), -2);

        case 7
            %% 12 trims: two speeds (0, 0.5 and 0.6 m/s), 12 steering
            steering = (-2:0.5:2) * pi / 18;
            ntrims = numel(steering) + 3;
            trim_inputs = zeros(ntrims, 2);
            trim_inputs(2, 1) = -0.6;
            trim_inputs(2, 2) = 0.05;
            trim_inputs(end, 1) = 0.6;
            trim_inputs(end, 2) = 0.05;
            trim_inputs(3:end - 1, 1) = steering;
            trim_inputs(3:end - 1, 2) = 0.6;
            trim_adjacency = ones(ntrims);
            % equilibrium is state 1
            % equilibrium is reachable from all states
            % and can reach all states, so all 1s is good
            % other states are connected by one hop
            trim_adjacency(2:end, 2:end) = trim_adjacency(2:end, 2:end) ...
                - triu(ones(ntrims - 1), 2) ...
                - tril(ones(ntrims - 1), -2);

        case 8
            %% 21 trims: four speeds (0, 0.05, 0.6 and 1.0 m/s), 12 steering
            steering = [-2, -2, -1.5, -1.5, -1, -1, -0.5, -0.5, 0, 0, 0.5, 0.5, 1, 1, 1.5, 1.5, 2, 2] * pi / 18;
            ntrims = numel(steering) + 3;
            trim_inputs = zeros(ntrims, 2);
            trim_inputs(2, 1) = -0.6;
            trim_inputs(2, 2) = 0.05;
            trim_inputs(end, 1) = 0.6;
            trim_inputs(end, 2) = 0.05;
            trim_inputs(3:end - 1, 1) = steering;
            trim_inputs(3:end - 1, 2) = [0.6, 1.0, 0.6, 1.0, 0.6, 1.0, 0.6, 1.0, 0.6, 1.0, 0.6, 1.0, 0.6, 1.0, 0.6, 1.0, 0.6, 1.0];
            trim_adjacency = ones(ntrims);
            % equilibrium is state 1
            % equilibrium is reachable from all states
            % and can reach all states, so all 1s is good
            % other states are connected by one hop
            trim_adjacency(2:end, 2:end) = trim_adjacency(2:end, 2:end) ...
                - triu(ones(ntrims - 1), 3) ...
                - tril(ones(ntrims - 1), -4);
        case 9
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

        case 10
            %% 12 trims: two speeds (0, 0.5 and 2.0 m/s), 12 steering
            steering = (-2:0.5:2) * pi / 18;
            ntrims = numel(steering) + 3;
            trim_inputs = zeros(ntrims, 2);
            trim_inputs(2, 1) = -0.6;
            trim_inputs(2, 2) = 0.05;
            trim_inputs(end, 1) = 0.6;
            trim_inputs(end, 2) = 0.05;
            trim_inputs(3:end - 1, 1) = steering;
            trim_inputs(3:end - 1, 2) = 2.0;
            trim_adjacency = ones(ntrims);
            % equilibrium is state 1
            % equilibrium is reachable from all states
            % and can reach all states, so all 1s is good
            % other states are connected by one hop
            trim_adjacency(2:end, 2:end) = trim_adjacency(2:end, 2:end) ...
                - triu(ones(ntrims - 1), 2) ...
                - tril(ones(ntrims - 1), -2);
        case 11
            %% 4 trims: two speeds (0 and 0.1 m/s), three steering (-pi/18 and pi/18)
            steering = (-1:1) * pi / 9;
            ntrims = numel(steering) + 1;
            trim_inputs = zeros(ntrims, 2);
            trim_inputs(2:end, 1) = steering;
            trim_inputs(2:end, 2) = 0.1;
            % All states connected
            trim_adjacency = ones(ntrims);
        case 12
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

            % add one aditional trim with speed v_max/2 and steering angle 0
            trim_inputs(ntrims + 1, 1) = 0;
            trim_inputs(ntrims + 1, 2) = v_max / 2;
            trim_adjacency = blkdiag(trim_adjacency, 1);
            trim_adjacency(end, :) = 1;
            trim_adjacency(:, end) = 1;
        case 13
            %% 3 trims (test)
            speed_max = 0.8;
            angle_max = pi / 6;
            trim_inputs = [0, 0;
                           -angle_max, speed_max;
                           0, speed_max;
                           angle_max, speed_max];
            trim_adjacency = 0.5 * eye(size(trim_inputs, 1));
            trim_adjacency(1, [2, 3, 4]) = 1;
            trim_adjacency(2, 3) = 1;
            trim_adjacency(3, 4) = 1;
            trim_adjacency = trim_adjacency + trim_adjacency';

        case 14 % acceleration constraint + increased steering at low speeds + double d_steering
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
            d_steer_max = 2.05 * d_steer;
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

        case 15 % acceleration constraint + increased steering at low speeds
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

        case 16 % acceleration constraint
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
            steer_max_lo_speed = d_steer * round((2 * pi / 18) / d_steer);
            steer_max_hi_speed = d_steer * round((2 * pi / 18) / d_steer);
            d_steer_max = 1.05 * d_steer;
            assert(d_steer < d_steer_max);
            steer_cla = cell(n_speeds, 1);

            steer_cla{1} = -steer_max_lo_speed:d_steer:steer_max_lo_speed;
            x = [speed_min + d_speed, speed_vec(n_speeds)];
            v = [steer_max_lo_speed, steer_max_hi_speed];
            % interpolate between hi and low steering angle
            for i_speed = 2:n_speeds
                xq = speed_vec(i_speed);
                max_steer = interp1(x, v, xq);
                max_steer = d_steer * round(max_steer / d_steer);
                min_steer = -max_steer;
                steer_cla{i_speed} = min_steer:d_steer:max_steer;
            end

            [trim_inputs, trim_adjacency] = build_mpa( ...
                speed_vec ...
                , steer_cla ...
                , acc_max ...
                , dec_max ...
                , d_steer_max ...
            );

        otherwise
            error('unknown mpa trim id');
    end

end

%% local function
function visualize_trims(trim_inputs, trim_adjacency)
    figure
    angle = rad2deg(trim_inputs(:, 1));
    speed = trim_inputs(:, 2);
    G = digraph(trim_adjacency, 'omitSelfLoops');
    p = plot(G, 'XData', angle, 'YData', speed, 'MarkerSize', 10);

    % workaround to change font size of node names
    p.NodeLabel = {};
    node_names = 1:length(angle);

    for i = 1:length(node_names)
        text(p.XData(i) + 0.8, p.YData(i), num2str(node_names(i)), 'FontSize', 20);
    end

    %
    xlim([-40, 40])
    ylim([-0.1, 1.0])
    xlabel('Steering Angle $\delta\:[\circ]$', 'Interpreter', 'latex', 'FontName', 'Times New Roman');
    ylabel('Speed $\nu\:[m/s]$', 'Interpreter', 'latex', 'FontName', 'Times New Roman');
    grid on
end
