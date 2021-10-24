function yaw = calculate_yaw(iveh)

    refPath = generateRefPath(iveh);
    nrefPoints = length(refPath);
    yaw = zeros(nrefPoints,1);
    
    
    dx1 = refPath(2,1)-refPath(1,1);
    dy1 = refPath(2,2)-refPath(1,2);
    
    if dx1 < 0
        yaw(1,1) = atan(dy1/dx1) + pi;
    else
        yaw(1,1) = atan(dy1/dx1);
    end
    
    for i = 2: nrefPoints-1
        dxi = refPath(i+1,1)-refPath(i-1,1);
        dyi = refPath(i+1,2)-refPath(i-1,2);
        if dxi < 0
            yaw(i,1) = atan(dyi/dxi) + pi;
        else
            yaw(i,1) = atan(dyi/dxi);
        end
    end
    dx_end = refPath(nrefPoints,1)-refPath(nrefPoints-1,1);
    dy_end = refPath(nrefPoints,2)-refPath(nrefPoints-1,2);
        
    if dx_end < 0
        yaw(nrefPoints,1) = atan(dy_end/dx_end) + pi;
    else
        yaw(nrefPoints,1) = atan(dy_end/dx_end);
    end
   
end