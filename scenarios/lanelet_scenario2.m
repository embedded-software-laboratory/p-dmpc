function scenario = lanelet_scenario2(isPB)
% LANELET_SCENARIO2   Constructor for scenario with two vehicles at an intersection one driving left and one straight.

    c = 0.2;

    scenario = Scenario();
    [lanelets, collision] = intersection_lanelets();
    scenario.lanelets = cellfun(@(S) c*S(:,:), lanelets, 'Uniform', 0);
    
    scenario.vehicle_to_lanelet = [     7, 12, 4;   ...
                                        5, 19, 8    ];
    
    veh = Vehicle();
    veh.trim_config = 1;
    veh.x_start = c*-18;
    veh.y_start = c*-2.5;
    veh.yaw_start = 0;
    veh.x_goal = c*18;
    veh.y_goal = c*-2.5;
    veh.yaw_goal = 0;
    veh_lanelets = vertcat(scenario.lanelets{scenario.vehicle_to_lanelet(1,:)});
    veh.referenceTrajectory = unique([veh_lanelets(:,LaneletInfo.cx),veh_lanelets(:,LaneletInfo.cy)],'rows','stable');
    scenario.vehicles = [scenario.vehicles, veh];
    
    veh = Vehicle();
    veh.trim_config = 1;
    veh.x_start = c*2.5;
    veh.y_start = c*-18;
    veh.yaw_start = 0.5*pi;
    veh.x_goal = c*-18;
    veh.y_goal = c*2.5;
    veh.yaw_goal = pi;
    veh_lanelets = vertcat(scenario.lanelets{scenario.vehicle_to_lanelet(2,:)});
    veh.referenceTrajectory = unique([veh_lanelets(:,LaneletInfo.cx),veh_lanelets(:,LaneletInfo.cy)],'rows','stable');
    scenario.vehicles = [scenario.vehicles, veh];

    scenario.plot_limits = c*[-20,20;-20,20];
    scenario.nVeh = numel(scenario.vehicles);
    scenario.name = sprintf('%i-intersection', scenario.nVeh);

    scenario.model = BicycleModel(veh.Lf,veh.Lr);

    scenario.T_end = 10;
   
    nVeh_mpa = scenario.nVeh;
    
    if isPB
       scenario.adjacency = coupling_adjacency_lanelets(scenario.vehicle_to_lanelet, collision);
       scenario.assignPrios = true;
       scenario.controller_name = strcat(scenario.controller_name, '-PB');
       scenario.controller = @(s,i) pb_controller(s,i);

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