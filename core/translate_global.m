function [x_globals,y_globals] = translate_global(yaw, x0, y0, x_locals, y_locals)
    
    dim = length(x_locals);
    
    % init vector
    x_globals = zeros(1,dim);
    y_globals = zeros(1,dim);
    
    % rotation matrix for local -> global translation
    R = [   cos(yaw) -sin(yaw)  ; ...
            sin(yaw) cos(yaw)   ];
        
    for i = 1:dim
        
        % local vector
        xy_vec = [x_locals(i) ; y_locals(i)];
        
        % rotated vector according to yaw angle
        xy_vec_rot = R*xy_vec;
        
        % add to initial position
        x_globals(i) = x0 + xy_vec_rot(1);
        y_globals(i) = y0 + xy_vec_rot(2);
        
    end
    
end
