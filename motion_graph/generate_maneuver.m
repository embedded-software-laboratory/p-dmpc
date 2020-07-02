% genrates maneuver struct
function maneuver = generate_maneuver(model, trim1, trim2, dt)
    
    maneuver = struct('start_trim',0,'end_trim',0,'yaw_change',0,'dx',0,'dy',0);
    
    L = model.Lr+model.Lf;
    
    % save trims connected by this maneuver
    maneuver.start_trim = trim1;
    maneuver.end_trim = trim2;
    
    % calculate displacement
    velocity_change = trim2.velocity -  trim1.velocity;
    acceleration = velocity_change/dt;
    
    steering_angle_change = trim2.steering - trim1.steering;
    steering_derivative = steering_angle_change/dt;
    
    % maneuver state changes in local coordinates
    %  TODO -- not sure if steering_angle_change or new steering_angle here --
    x0 = zeros(model.nx, 1);
    x0(4) = trim1.velocity;
    x0(5) = trim1.steering;
    [t, x] = ode45(@(t, x) model.ode(x, [steering_derivative,acceleration]), ...
                   [0 dt], ...
                   x0, ...
                   odeset('RelTol',1e-8,'AbsTol',1e-8));
   
    % get index of last iteration of ode
    last = length(x);
    
    % assign values to struct 
    maneuver.yaw_change = x(last,3);
    maneuver.dx = x(last,1);
    maneuver.dy = x(last,2);
    
end
