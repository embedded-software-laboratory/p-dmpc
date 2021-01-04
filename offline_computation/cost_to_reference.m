function distance = cost_to_reference

    motion_graph = create_motion_graph_list('trim_set_3_1', 1);
    
    angle = [0; pi/6; pi/3; pi/2];
    avg_distance = [];
    dx = [];
    dy = [];
    iterations = [];
    
    for i = 1:length(angle)
        
        yaw = angle(i);
        x = 0;
        y = 0;
        distance = 0;
        n_maneuver = 0;
        
        maneuver = motion_graph.maneuvers{1, 3};
        [x, y] = translate_global(yaw, x, y, maneuver.dx, maneuver.dy);
        distance = distance + y;
        yaw = yaw + maneuver.dyaw;
        n_maneuver = n_maneuver + 1;

        maneuver = motion_graph.maneuvers{3, 1};
        [x, y] = translate_global(yaw, x, y, maneuver.dx, maneuver.dy);
        distance = distance + y;
        yaw = yaw + maneuver.dyaw;
        n_maneuver = n_maneuver + 1;

        maneuver = motion_graph.maneuvers{3, 3};
        while(abs(yaw + pi / 2) > 0.15)
            [x, y] = translate_global(yaw, x, y, maneuver.dx, maneuver.dy);
            distance = distance + y;
            yaw = yaw + maneuver.dyaw;
            n_maneuver = n_maneuver + 1;
        end

%         maneuver = motion_graph.maneuvers{1, 2};
%         [x, y] = translate_global(yaw, x, y, maneuver.dx, maneuver.dy);
%         distance = distance + y;
%         yaw = yaw + maneuver.dyaw;
%         n_maneuver = n_maneuver + 1;
% 
%         maneuver = motion_graph.maneuvers{2, 1};
%         [x, y] = translate_global(yaw, x, y, maneuver.dx, maneuver.dy);
%         distance = distance + y;
%         yaw = yaw + maneuver.dyaw;
%         n_maneuver = n_maneuver + 1;
% 
%         maneuver = motion_graph.maneuvers{2, 2};
%         while(abs(yaw) > 0.15)
%             [x, y] = translate_global(yaw, x, y, maneuver.dx, maneuver.dy);
%             distance = distance + y;
%             yaw = yaw + maneuver.dyaw;
%             n_maneuver = n_maneuver + 1;
%         end
        
    avg_distance = [avg_distance; distance / n_maneuver];
    dx = [dx; x];
    dy = [dy; y];
    iterations = [iterations; n_maneuver];
    end
    
    situation_costs = table(angle, dx, dy, avg_distance, iterations);
    
    save('offline_computation/situation_costs.mat', 'situation_costs');
end

