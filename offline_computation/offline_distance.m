function offline_distance(motion_graph)

    maneuvers = motion_graph.motionGraphList.maneuvers;
    angles = [];
    while true
        angles = [angles, maneuvers{1, 2}.dyaw];
    end

    angles = [15, 30, 45, 60, 75, 90];
    angles = deg2rad(angles);

    for angle = angles 
        
    end
    
end

