function scenario = commonroad(vehicle_ids,options)
% Commonroad_Scenario   

    scenario = Scenario();
    scenario.name = 'Commonroad';
    scenario.trim_set = 12;
    scenario.dt = 0.2;
    scenario.T_end = 30;
    scenario.Hp = 6;

    % get road data
    [scenario.lanelets, scenario.adjacency_lanelets, scenario.semi_adjacency_lanelets,...
        scenario.intersection_lanelets, scenario.lanelet_boundary, scenario.road_raw_data, scenario.lanelet_relationships] = get_road_data();
    
    nVeh = options.amount;
    
    for iveh = 1:nVeh
        
        veh = Vehicle();
        veh.ID = vehicle_ids(iveh); % vehicle ID
        veh.trim_config = 1;
        ref_path = generate_ref_path(veh.ID, scenario.lanelets);% function to generate refpath based on CPM Lab road geometry
        refPath = ref_path.path;
        veh.x_start = refPath(1,1);
        veh.y_start = refPath(1,2);
        veh.x_goal = refPath(2:end,1);
        veh.y_goal = refPath(2:end,2);
        
        veh.referenceTrajectory = [veh.x_start veh.y_start
                                   veh.x_goal  veh.y_goal];
        veh.lanelets_index = ref_path.lanelets_index;
        veh.points_index = ref_path.points_index;

        yaw = calculate_yaw(refPath);
        veh.yaw_start = yaw(1);
        veh.yaw_goal = yaw(2:end);
        scenario.vehicles = [scenario.vehicles, veh];
    end

    scenario.plot_limits = [0,4.5;0,4];  
    scenario.nVeh = nVeh;
    scenario.model = BicycleModel(veh.Lf,veh.Lr);
    nVeh_mpa = scenario.nVeh;
    
    scenario.name = options.scenario;
    scenario.priority_option = options.priority;
    scenario.isParl = options.isParl;
    
    if options.isPB 
       scenario.adjacency = zeros(nVeh,nVeh);
       scenario.assignPrios = true;
       nVeh_mpa = 1;

       if options.isParl
            scenario.controller_name = strcat(scenario.controller_name, '-parallel computation');
            scenario.controller = @(s,i) pb_controller_parl(s,i);
       else
           scenario.controller_name = strcat(scenario.controller_name, '-PB');
           scenario.controller = @(s,i) pb_controller(s,i);
       end

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
        , scenario.is_allow_non_convex...
        , options...
    );
%     plot_local_reachable_sets(scenario.mpa, scenario.is_allow_non_convex)
end
