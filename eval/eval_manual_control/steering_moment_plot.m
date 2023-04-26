function steering_moment_plot()
    % \tau_{s}(t) =   & - I_{\text{sc}} \ddot{\steeringWheelAngle}(t) - b_{\text{ps}} \dot{\steeringWheelAngle}(t) \\
    %             & + k_1 \left(\frac{\varSpeed_y(t) + L_f \dot{\varOrientation}(t)}{\varSpeed_x(t)} - \delta(t) \right) - k_2 \delta(t),
    % From mehdizadeh2011new
    inertia_steering_column = 0.01;
    b_ps = 3;
    k1 = 300;
    k2 = 5;
    L_f = 1.3;
%     side_slip_angle = @(steering_angle_rad) atan( L_r / L * tan(steering_angle_rad) );
% 
%     orientation = @(speed, steering_angle_rad) speed ./ L .* tan(steering_angle_rad) .* cos(side_slip_angle(steering_angle_rad));
%     v_x = @(speed, steering_angle_rad) speed .* cos ( orientation(speed, steering_angle_rad) + side_slip_angle(steering_angle_rad) );
%     v_y = @(speed, steering_angle_rad) speed .* sin ( orientation(speed, steering_angle_rad) + side_slip_angle(steering_angle_rad) );

    torque_mehdizadeh = @(speed, steering_angle_rad, steering_wheel_angle_speed, steering_wheel_angle_acceleration) ...
        - inertia_steering_column * steering_wheel_angle_acceleration ...
        - b_ps * steering_wheel_angle_speed ...
        + k1 * ((v_y(speed, steering_angle_rad) + L_f .* yaw_rate(speed, steering_angle_rad)) ./ v_x(speed, steering_angle_rad) - steering_angle_rad) ...
        - k2 * steering_angle_rad;

%     gridded_speed_and_steering_angle(torque_mehdizadeh);
    gridded_speed_and_steering_angle(@torque_1);
    gridded_speed_and_steering_angle(@torque_2);
    gridded_speed_and_steering_angle(@torque_3);
    gridded_steering_wheel();

    function gridded_speed_and_steering_angle(torque_function)
        steering_wheel_angle_speed = 0;
        steering_wheel_angle_acceleration = 0;
        speed_max = 40;
        steering_angle_max_rad = 36/180 * pi;

        [speed_grid, steering_angle_grid] = meshgrid(linspace(0,speed_max,100), linspace(0,steering_angle_max_rad,50));
        torque_grid = torque_function(speed_grid, steering_angle_grid, steering_wheel_angle_speed, steering_wheel_angle_acceleration);

        figure
        surf(speed_grid, steering_angle_grid, torque_grid)
        zlim([-0.5 1])
    end
    
    function gridded_steering_wheel()
    end
end

function result = side_slip_angle(steering_angle_rad)
L_f = 1.3;
    L_r = L_f;
    L = L_r + L_f;
    result = atan( L_r / L * tan(steering_angle_rad) );
end

function result = yaw_rate(speed, steering_angle_rad)
L_f = 1.3;
    L_r = L_f;
    L = L_r + L_f;
    result = speed ./ L .* tan(steering_angle_rad) .* cos(side_slip_angle(steering_angle_rad));
end

function result = v_x(speed, steering_angle_rad)
    yaw = 0;
    result = speed .* cos ( yaw + side_slip_angle(steering_angle_rad) );
end

function result = v_y(speed, steering_angle_rad)
    yaw = 0;
    result = speed .* sin( yaw + side_slip_angle(steering_angle_rad) );
end

function result = torque_1(speed, steering_angle_rad, ~, ~)
    speed_max = 40;
    steering_angle_max_rad = 36/180 * pi;
    torque_standstill = 0.1;
    speed_min_torque = 1.5;
    % speed < speed_min_torque
        % sign( steering_angle * steering_speed ) >= 0
            % 0.2 + torque_standstill .* (1-(speed ./speed_min_torque)).^3
            % towards middle
        % sign( steering_angle * steering_speed ) < 0
            % 0.2 - (torque_standstill + 0.4) .* (1-(speed ./speed_min_torque)).^3
            % towards outside
    torque_slow = 0.2 + 0.3 .* exp(-(speed ./speed_min_torque));
%     torque_slow_neg = ;
    result = -torque_slow +  ...
        0.8 * (speed ./speed_max).^1.5 ...
        .* (steering_angle_rad ./ steering_angle_max_rad);
end

function result = torque_2(speed, steering_angle_rad, ~, ~)
v_max = 40;
    result = 0.1 ...
    + 0.4 .* abs(steering_angle_rad) .* sin(speed ./ v_max .* pi / 2) ...
        + 0.5 .* sin(speed ./ v_max .* pi ./ 2);
end

function result = torque_3(speed, steering_angle_rad, ~, ~)
speed_max = 40;
steering_angle_max_rad = 0.2*pi;
    result = 0.2 ...
        + 0.8 * (speed ./speed_max).^1.5 .* ...
        (0.3 + 0.7*(steering_angle_rad ./ steering_angle_max_rad).^1.3);
    % TODO Torque at standstill. v<1.8 m/s
end