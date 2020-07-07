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
    [x_next,y_next,yaw_next] = maneuver_displacement(model, 0, 0, 0, trim1.velocity, trim1.steering, steering_angle_change, velocity_change, dt);
    
    maneuver.dx = x_next;
    maneuver.dy = y_next;
    maneuver.dyaw = yaw_next;
   
    area = calculate_maneuver_area(model,maneuver);
    
    maneuver.area = area;
    
end
