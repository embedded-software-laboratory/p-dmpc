function scenario = commonroad(options,vehicle_ids,mVehid,m2Vehid,is_sim_lab)
% Commonroad_Scenario   

    nVeh = options.amount;
    scenario = Scenario();
    scenario.name = 'Commonroad';
    scenario.trim_set = 12;
    scenario.dt = 0.2;
    scenario.options = options; 

    % get road data
    road_data = RoadData().get_road_data();
    scenario.lanelets = road_data.lanelets;
    scenario.adjacency_lanelets = road_data.adjacency_lanelets;
    scenario.semi_adjacency_lanelets = road_data.semi_adjacency_lanelets;
    scenario.intersection_lanelets = road_data.intersection_lanelets;
    scenario.lanelet_boundary = road_data.lanelet_boundary;
    scenario.road_raw_data = road_data.road_raw_data;
    scenario.lanelet_relationships  = road_data.lanelet_relationships;

    scenario.T_end = 20;
    scenario.Hp = 6;
    
    for iveh = 1:nVeh
        
        veh = Vehicle();
        veh.trim_config = 1;

        if is_sim_lab
            ref_path = generate_ref_path(vehicle_ids(iveh), scenario.lanelets);% function to generate refpath based on CPM Lab road geometry
            %[ref_path, scenario] = generate_random_path(scenario, vehicle_ids(iveh), 20, (vehicle_ids(iveh)+31));
        else
            if (mVehid == vehicle_ids(iveh) || m2Vehid == vehicle_ids(iveh))
                [ref_path, scenario] = generate_manual_path(scenario, vehicle_ids(iveh), 3, (vehicle_ids(iveh)+31), false);  
                %ref_path = generate_manual_path(scenario, vehicle_ids(iveh), 3, (vehicle_idsd(iveh)));     
            else
                [ref_path, scenario] = generate_random_path(scenario, vehicle_ids(iveh), 3, (vehicle_ids(iveh)+31)); % function to generate random path for autonomous vehicles based on CPM Lab road geometry
                %ref_path = generate_random_path(scenario, vehicle_ids(iveh), 3, (vehicle_ids(iveh)));
            end
        end
        
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

    % initialize speed profile vector, currently 3 speed profiles are available
    scenario.speed_profile_mpas = [scenario.mpa, scenario.mpa, scenario.mpa];
 
%     plot_local_reachable_sets(scenario.mpa, scenario.is_allow_non_convex)
end
