% genrates maneuver struct
function maneuver = generate_maneuver(model, trim1, trim2, dt)
    
    % length of vehicle
    L = model.Lr+model.Lf;
    
    % save trims connected by this maneuver
    maneuver.start_trim = trim1;
    maneuver.end_trim = trim2;
    
    % calculate displacement
    velocity_change = trim2.velocity -  trim1.velocity;
    
    steering_angle_change = trim2.steering - trim1.steering;
    
    % module to calculate displacement
    [xs,ys,yaws] = maneuver_displacement(model, 0, 0, 0, trim1.velocity, trim1.steering, steering_angle_change, velocity_change, dt);
    
    last = length(xs);
    
    % assign values to struct 
    x_next = xs(last);
    y_next = ys(last);
    yaw_next = yaws(last);
    
    maneuver.dx = x_next;
    maneuver.dy = y_next;
    maneuver.dyaw = yaw_next;
    
    maneuver.xs = xs;
    maneuver.ys = ys;
    maneuver.yaws = yaws;
   
    area = calculate_maneuver_area(model,maneuver);
    
    maneuver.area = area;
    
end
