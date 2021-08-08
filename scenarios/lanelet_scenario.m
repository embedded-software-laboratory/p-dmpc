function scenario = lanelet_scenario(isPB)
% LANELET_SCENARIO   Constructor for scenario with two vehicles at an intersection one driving right and one straight.

    c = 0.2;

    scenario = Scenario();
    scenario.lanelets = cellfun(@(S) c*S(:,:), intersection_lanelets(), 'Uniform', 0);
    
    scenario.vehicle_to_lanelet = [     1, 9, 6;    ...
                                        3, 14, 2        ];
    
    veh = Vehicle();
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
    veh.trim_config = 1;
    veh.x_start = c*18;
    veh.y_start = c*2.5;
    veh.yaw_start = pi;
    veh.x_goal = c*2.5;
    veh.y_goal = c*18;
    veh.yaw_goal = 0.5*pi;
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
        % undirected coupling adjacency is complete
        scenario.adjacency = zeros(scenario.nVeh);
       if scenario.assignPrios
            scenario.directed_coupling = [];
       else
            scenario.directed_coupling = zeros(scenario.nVeh);
       end
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
    );
end