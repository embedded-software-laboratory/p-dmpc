function scenario = lanelet_scenario4(isPB,isParl,isROS)
% LANELET_SCENARIO4   Constructor for scenario with four vehicles at an intersection driving straight
    c = 0.2;

    scenario = Scenario();
    [lanelets, collision] = intersection_lanelets();
    scenario.lanelets = cellfun(@(S) c*S(:,:), lanelets, 'Uniform', 0);
    
    scenario.vehicle_to_lanelet = [     1, 9, 6;   ...
                                        5, 11, 2;  ...
                                        3, 10, 8;  ...
                                        7, 12, 4    ];
    
    veh = Vehicle();
    veh.ID = 1;
    veh.trim_config = 1;
    veh.x_start = c*-2.5;
    veh.y_start = c*18;
    veh.yaw_start = 1.5*pi;
    veh.x_goal = c*-2.5;
    veh.y_goal = c*-18;
    veh.yaw_goal = 1.5*pi;
    veh_lanelets = vertcat(scenario.lanelets{scenario.vehicle_to_lanelet(1,:)});
    veh.referenceTrajectory = unique([veh_lanelets(:,LaneletInfo.cx),veh_lanelets(:,LaneletInfo.cy)],'rows','stable');
    scenario.vehicles = [scenario.vehicles, veh];
    
    veh = Vehicle();
    veh.ID = 2;
    veh.trim_config = 1;
    veh.x_start = c*2.5;
    veh.y_start = c*-18;
    veh.yaw_start = 0.5*pi;
    veh.x_goal = c*2.5;
    veh.y_goal = c*18;
    veh.yaw_goal = 0.5*pi;
    veh_lanelets = vertcat(scenario.lanelets{scenario.vehicle_to_lanelet(2,:)});
    veh.referenceTrajectory = unique([veh_lanelets(:,LaneletInfo.cx),veh_lanelets(:,LaneletInfo.cy)],'rows','stable');
    scenario.vehicles = [scenario.vehicles, veh];
    
    veh = Vehicle();
    veh.ID = 3;
    veh.trim_config = 1;
    veh.x_start = c*13;
    veh.y_start = c*2.5;
    veh.yaw_start = pi;
    veh.x_goal = c*-18;
    veh.y_goal = c*2.5;
    veh.yaw_goal = pi;
    veh_lanelets = vertcat(scenario.lanelets{scenario.vehicle_to_lanelet(3,:)});
    veh.referenceTrajectory = unique([veh_lanelets(:,LaneletInfo.cx),veh_lanelets(:,LaneletInfo.cy)],'rows','stable');
    scenario.vehicles = [scenario.vehicles, veh];
    
    veh = Vehicle();
    veh.ID = 4;
    veh.trim_config = 1;
    veh.x_start = c*-13;
    veh.y_start = c*-2.5;
    veh.yaw_start = 0;
    veh.x_goal = c*18;
    veh.y_goal = c*-2.5;
    veh.yaw_goal = 0;
    veh_lanelets = vertcat(scenario.lanelets{scenario.vehicle_to_lanelet(4,:)});
    veh.referenceTrajectory = unique([veh_lanelets(:,LaneletInfo.cx),veh_lanelets(:,LaneletInfo.cy)],'rows','stable');
    scenario.vehicles = [scenario.vehicles, veh];

    scenario.plot_limits = c*[-20,20;-20,20];
    scenario.nVeh = numel(scenario.vehicles);
    scenario.name = sprintf('%i-intersection', scenario.nVeh);

    scenario.model = BicycleModel(veh.Lf,veh.Lr);

   
    nVeh_mpa = scenario.nVeh;
    scenario.isParl = logical(isParl); % is use parallel computation
    
    if isPB
       scenario.adjacency = coupling_adjacency_lanelets(scenario.vehicle_to_lanelet, collision);
       scenario.assignPrios = true;
       scenario.controller_name = strcat(scenario.controller_name, '-PB');

       if scenario.isParl % use parallel trajectory planning
           scenario.controller = @(s,i) pb_controller_parl(s,i);
           if isROS
               scenario.controller = @(s,i) pb_controller_parl_ROS(s,i);
           end
       else % use sequential trajectory planning
           scenario.controller = @(s,i) pb_controller(s,i);
       end

       nVeh_mpa = 1;
    end

    recursive_feasibility = true;
    scenario.mpa = MotionPrimitiveAutomaton(...
        scenario.model...
        , scenario.trim_set...
        , scenario.offset...
        , scenario.dt...
        , nVeh_mpa...
        , scenario.Hp...
        , scenario.tick_per_step...
        , recursive_feasibility...
        , options...
    );
    

end