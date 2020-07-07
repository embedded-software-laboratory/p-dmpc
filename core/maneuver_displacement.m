function [x_next, y_next, yaw_next] = maneuver_displacement(model, x_init, y_init, yaw_init, v_init, delta_init, steering_angle_change, velocity_change, dt)
    
    steering_derivative = steering_angle_change/dt;
    acceleration = velocity_change/dt;
    
    % maneuver state changes in local coordinates
    x0 = zeros(model.nx, 1);
    x0(1) = x_init;
    x0(2) = y_init;
    x0(3) = yaw_init;
    x0(4) = v_init;
    x0(5) = delta_init;
    
    [t, x] = ode45(@(t, x) model.ode(x, [steering_derivative,acceleration]), ...
                   [0 dt], ...
                   x0, ...
                   odeset('RelTol',1e-8,'AbsTol',1e-8));
   
    % get index of last iteration of ode
    last = length(x);
    
    % assign values to struct 
    x_next = x(last,1);
    y_next = x(last,2);
    yaw_next = x(last,3);

end
