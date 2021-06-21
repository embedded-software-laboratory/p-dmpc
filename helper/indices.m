function idx=indices
    % Structure of integer constants used as indices in vectors.

    idx = struct;
    
    % General
    idx.x = 1;
    idx.y = 2;
    idx.heading = 3;
    idx.speed = 4;
    
    % Vehicle
    idx.acceleration = 5;
    
    % Obstacle
    idx.length = 5;
    idx.width = 6;
    
end

