function maneuver_area = calculate_maneuver_area(model,maneuver)
    
    hw = model.W/2;
    
    % collision offset
    offset = 0.1;
    
    % local rectangel of bycicle model [bd bu fu fd]
    x_rec = [-model.Lr-offset -model.Lr-offset model.Lf+offset model.Lf+offset]; 
    y_rec = [-hw-offset hw+offset hw+offset -hw-offset];
        
    % calculate displacement of model shape
    [x_rec_new, y_rec_new] = translate_global(maneuver.dyaw, maneuver.dx, maneuver.dy, x_rec, y_rec);
    
    signum = sign(maneuver.dyaw);
    
    switch signum
        case 0
            lastX = x_rec_new(4);
            lastY = y_rec_new(4);
        case 1
            lastX = x_rec_new(4);
            lastY = y_rec(4);
        case -1
            lastX = x_rec_new(3);
            lastY = y_rec(3);
    end
    
    maneuver_area = [   x_rec(1) x_rec(2) x_rec_new(3) x_rec_new(4) lastX; ...
                        y_rec(1) y_rec(2) y_rec_new(3) y_rec_new(4) lastY   ];
                    
    % --- HINT ---
    % for maneuver polygon
    % pgon = convhull(polyshape(maneuver_area(1,:),maneuver_area(2,:));
    
end

