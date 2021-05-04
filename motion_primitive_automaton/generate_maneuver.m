% genrates maneuver struct
function maneuver = generate_maneuver(model, trim1, trim2, offset, dt)
    % TODO make independent from model, currently only works for BicycleModel
    %% calculate displacement
    
    steering_derivative = (trim2.steering - trim1.steering) / dt;
    acceleration = (trim2.speed - trim1.speed) / dt;
    
    % maneuver state changes in local coordinates
    x0 = zeros(model.nx, 1);
    x0(4) = trim1.speed;
    if model.nx > 4
        x0(5) = trim1.steering;
    end
    
    [~, x] = ode45(@(t, x) model.ode(x, [steering_derivative,acceleration]), ...
                   [0 dt], ...
                   x0, ...
                   odeset('RelTol',1e-8,'AbsTol',1e-8)...
    );
     
    % assign values to struct 
    maneuver.xs = x(:,1)';
    maneuver.ys = x(:,2)';
    maneuver.yaws = x(:,3)';

    maneuver.dx = maneuver.xs(end);
    maneuver.dy = maneuver.ys(end);
    maneuver.dyaw = maneuver.yaws(end);
    
    
    %% Area
    % local rectangle of bycicle model [coordinates clockwise from lower left corner]
    veh = Vehicle();
    x_rec1 = [-1, -1,  1,  1] * (veh.Length/2+offset);
    y_rec1 = [-1,  1,  1, -1] * (veh.Width/2+offset);
        
    % calculate displacement of model shape
    [x_rec2, y_rec2] = translate_global(maneuver.dyaw, maneuver.dx, maneuver.dy, x_rec1, y_rec1);
    
    signum = sign(maneuver.dyaw);
    
    switch signum
        case 0
            maneuver_area = [   x_rec1(1) x_rec1(2) x_rec2(3) x_rec2(4); ...
                                y_rec1(1) y_rec1(2) y_rec2(3) y_rec2(4)   ];
        case 1
            lastX = x_rec2(4);
            lastY = y_rec1(4);
            maneuver_area = [   x_rec1(1) x_rec1(2) x_rec2(3) x_rec2(4) lastX; ...
                                y_rec1(1) y_rec1(2) y_rec2(3) y_rec2(4) lastY   ];
        case -1
            lastX = x_rec2(3);
            lastY = y_rec1(3);
            maneuver_area = [   x_rec1(1) x_rec1(2) lastX x_rec2(3) x_rec2(4); ...
                                y_rec1(1) y_rec1(2) lastY y_rec2(3) y_rec2(4)   ];
    end
    
    maneuver.area = maneuver_area;
    
end
