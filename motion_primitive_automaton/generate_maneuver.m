function maneuver = generate_maneuver(model, trim1, trim2, offset, dt, nTicks, is_allow_non_convex)
% GENERATE_MANEUVER     Generate a maneuver for motion primitive automaton.

    % currently works for BicycleModel
    % calculate displacement
    
    steering_derivative = (trim2.steering - trim1.steering) / dt;
    acceleration = (trim2.speed - trim1.speed) / dt;
    
    % maneuver state changes in local coordinates
    x0 = zeros(model.nx, 1);
    x0(4) = trim1.speed;
    if model.nx > 4
        x0(5) = trim1.steering;
    end
    
    % 1 tick more per step cause first values are the same as last ones in preceeding step
    [~, x] = ode45(@(t, x) model.ode(x, [steering_derivative,acceleration]), ...
                   linspace(0,dt,nTicks+1), ...
                   x0, ...
                   odeset('RelTol',1e-8,'AbsTol',1e-8)...
    );
     
    maneuver = struct('xs',[],'ys',[],'yaws',[],'dx',[],'dy',[],'dyaw',[],'area',[],'area_without_offset',[],'area_large_offset',[]);
    % assign values to struct 
    maneuver.xs = x(:,1)';
    maneuver.ys = x(:,2)';
    maneuver.yaws = x(:,3)';

    maneuver.dx = maneuver.xs(end);
    maneuver.dy = maneuver.ys(end);
    maneuver.dyaw = maneuver.yaws(end);
    
    % local rectangle of bycicle model [coordinates clockwise from lower left corner]
    veh = Vehicle();

    % with normal offset
    x_rec1 = [-1, -1,  1,  1] * (veh.Length/2+offset);
    y_rec1 = [-1,  1,  1, -1] * (veh.Width/2+offset);
    % calculate displacement of model shape
    [x_rec2, y_rec2] = translate_global(maneuver.dyaw, maneuver.dx, maneuver.dy, x_rec1, y_rec1);
    signum = sign(maneuver.dyaw); % positive for left turn, negative for right turn
    maneuver.area = get_maneuver_area(x_rec1, y_rec1, x_rec2, y_rec2, signum, is_allow_non_convex);
    assert(all(maneuver.area(:,1)==maneuver.area(:,end))) % must be closed shape

    % without offset
    x_rec1_without_offset = [-1, -1,  1,  1] * (veh.Length/2);
    y_rec1_without_offset = [-1,  1,  1, -1] * (veh.Width/2);
    % calculate displacement of model shape
    [x_rec2_without_offset, y_rec2_without_offset] = translate_global(maneuver.dyaw, maneuver.dx, maneuver.dy, x_rec1_without_offset, y_rec1_without_offset);
    signum = sign(maneuver.dyaw);    
    maneuver.area_without_offset = get_maneuver_area(x_rec1_without_offset, y_rec1_without_offset, x_rec2_without_offset, y_rec2_without_offset, signum, is_allow_non_convex); 
    assert(all(maneuver.area_without_offset(:,1)==maneuver.area_without_offset(:,end))) % must be closed shape
    
    % with larger offset: for the last step of prediction horizon
    x_rec1_larger_offset = [-1, -1,  1,  1] * (veh.Length/2 + 0.05);
    y_rec1_larger_offset = [-1,  1,  1, -1] * (veh.Width/2 + 0.01);
    % calculate displacement of model shape
    [x_rec2_larger_offset, y_rec2_larger_offset] = translate_global(maneuver.dyaw, maneuver.dx, maneuver.dy, x_rec1_larger_offset, y_rec1_larger_offset);
    signum = sign(maneuver.dyaw);
    maneuver.area_large_offset = get_maneuver_area(x_rec1_larger_offset, y_rec1_larger_offset, x_rec2_larger_offset, y_rec2_larger_offset, signum, is_allow_non_convex);  
    assert(all(maneuver.area_large_offset(:,1)==maneuver.area_large_offset(:,end))) % must be closed shape
end


%% local function
function maneuver_area = get_maneuver_area(x_rec1, y_rec1, x_rec2, y_rec2, signum, is_allow_non_convex)
% Approximates the maneuver area beased the starting area and end area
% two different ways: 1. using non-convex polygon
%                     2. using convex polygon
%
    switch signum
        case 0 % go straight
            maneuver_area = [   x_rec1(1) x_rec1(2) x_rec2(3) x_rec2(4) x_rec1(1);
                                y_rec1(1) y_rec1(2) y_rec2(3) y_rec2(4) y_rec1(1)]; % repeat the first column to enclose the shape
        case 1 % turn left
            if is_allow_non_convex
                % non-convex polygon to approximate the maneuver area
                maneuver_area = [   x_rec1(1) x_rec1(2) x_rec2(2) x_rec2(3) x_rec2(4) x_rec1(4) x_rec1(1);
                                    y_rec1(1) y_rec1(2) y_rec2(2) y_rec2(3) y_rec2(4) y_rec1(4) y_rec1(1)];
            else
                % convex polygon to approximate the maneuver area
                lastX = x_rec2(4);
                lastY = y_rec1(4);
                maneuver_area = [   x_rec1(1) x_rec1(2) x_rec2(3) x_rec2(4) lastX x_rec1(1);
                                    y_rec1(1) y_rec1(2) y_rec2(3) y_rec2(4) lastY y_rec1(1)];
            end

        case -1 % turn right
            if is_allow_non_convex
                maneuver_area = [   x_rec1(1) x_rec1(2) x_rec1(3) x_rec2(3) x_rec2(4) x_rec2(1) x_rec1(1);
                                    y_rec1(1) y_rec1(2) y_rec1(3) y_rec2(3) y_rec2(4) y_rec2(1) y_rec1(1)];
            else
                lastX = x_rec2(3);
                lastY = y_rec1(3);
                maneuver_area = [   x_rec1(1) x_rec1(2) lastX x_rec2(3) x_rec2(4) x_rec1(1);
                                    y_rec1(1) y_rec1(2) lastY y_rec2(3) y_rec2(4) y_rec1(1)];
            end
    end
end