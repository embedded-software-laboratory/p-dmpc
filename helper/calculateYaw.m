function yaw = calculateYaw(iveh)

    refPath = generateRefPath(iveh);
    nrefPoints = length(refPath);
    yaw = zeros(nrefPoints,1);
    
    dx1 = refPath(2,2)-refPath(1,2);
    dy1 = refPath(2,1)-refPath(1,1);
    
    if dx1 < 0
        yaw(1,1) = atan(dx1/dy1) + pi;
    else
        yaw(1,1) = atan(dx1/dy1);
    end
    
    for i = 2: nrefPoints-1
        dxi = refPath(i+1,2)-refPath(i-1,2);
        dyi = refPath(i+1,1)-refPath(i-1,1);
        if dxi < 0
            yaw(i,1) = atan(dxi/dyi) + pi;
        else
            yaw(i,1) = atan(dxi/dyi);
        end
    end
    dx_end = refPath(nrefPoints,2)-refPath(nrefPoints-1,2);
    dy_end = refPath(nrefPoints,1)-refPath(nrefPoints-1,1);
        
    if dx_end < 0
        yaw(nrefPoints,1) = 0.8*(atan(dx_end/dy_end) + pi);
    else
        yaw(nrefPoints,1) = 0.8*(atan(dx_end/dy_end));
    end
   
end