function scenario = commonroad(options,vehicle_ids,mVehid,m2Vehid,is_sim_lab)
% Commonroad_Scenario   

    scenario = Scenario();

    options.recursive_feasibility = true;
    % read from optionos
    scenario.options = options; 

    options.is_allow_non_convex = true;

    scenario.name = 'Commonroad';

    % get road data
    road_data = RoadData().get_road_data();
    scenario.lanelets = road_data.lanelets;
    scenario.adjacency_lanelets = road_data.adjacency_lanelets;
    scenario.semi_adjacency_lanelets = road_data.semi_adjacency_lanelets;
    scenario.intersection_lanelets = road_data.intersection_lanelets;
    scenario.lanelet_boundary = road_data.lanelet_boundary;
    scenario.road_raw_data = road_data.road_raw_data;
    scenario.lanelet_relationships  = road_data.lanelet_relationships;
    
    nVeh = options.amount;
    for iveh = 1:nVeh
        
        veh = Vehicle();
        veh.trim_config = 1;

        if is_sim_lab || ~scenario.options.is_mixed_traffic
            ref_path = generate_ref_path_loop(vehicle_ids(iveh), scenario.lanelets);% function to generate refpath based on CPM Lab road geometry
            %[ref_path, scenario] = generate_random_path(scenario, vehicle_ids(iveh), 20, (vehicle_ids(iveh)+31));
        else
            if (mVehid == vehicle_ids(iveh) || m2Vehid == vehicle_ids(iveh))
                % function to generate random path for manual vehicles based on CPM Lab road geometry
                [ref_path, scenario] = generate_manual_path(scenario, vehicle_ids(iveh), 3, (vehicle_ids(iveh)+31), false);       
            else
                % function to generate random path for autonomous vehicles based on CPM Lab road geometry
                [ref_path, scenario, veh.lane_change_indices, veh.lane_change_lanes] = generate_random_path(scenario, vehicle_ids(iveh), 3, (vehicle_ids(iveh)+31));
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
%       % example in paper
%     scenario.vehicles(1).x_start = 2.0;
%     scenario.vehicles(1).y_start = 1.775;
%     scenario.vehicles(1).yaw_start = 0;
% 
%     scenario.vehicles(2).x_start = 2.32;
%     scenario.vehicles(2).y_start = 1.41;
%     scenario.vehicles(2).yaw_start = deg2rad(100);
% 
%     scenario.vehicles(3).x_start = 1.555;
%     scenario.vehicles(3).y_start = 1.775;
%     scenario.vehicles(3).yaw_start = 0;
% 
%     scenario.vehicles(4).x_start = 2.025;
%     scenario.vehicles(4).y_start = 2.48;
%     scenario.vehicles(4).yaw_start = deg2rad(270);
%     
%     scenario.vehicles(5).x_start = 2.175;
%     scenario.vehicles(5).y_start = 2.67;
%     scenario.vehicles(5).yaw_start = deg2rad(280);
% 
%     scenario.vehicles(6).x_start = 2.89;
%     scenario.vehicles(6).y_start = 2.075;
%     scenario.vehicles(6).yaw_start = deg2rad(180);

    scenario.options.plot_limits = [0,4.5;0,4];
    scenario.model = BicycleModel(veh.Lf,veh.Lr);
    
    scenario.name = options.scenario_name;
    
    if options.isPB 
       scenario.adjacency = zeros(nVeh,nVeh);
       scenario.assignPrios = true;

       if options.isParl
            % if parallel computation is used
            scenario.controller_name = strcat(scenario.controller_name, '-Parl');
            scenario.controller = @(s,i) pb_controller_parl(s,i);
       else
           scenario.controller_name = strcat(scenario.controller_name, '-PB');
           scenario.controller = @(s,i) pb_controller(s,i);
       end
    end

    
    scenario.mpa = MotionPrimitiveAutomaton(scenario.model, options);

    % initialize speed profile vector, currently 3 speed profiles are available
    scenario.speed_profile_mpas = [scenario.mpa, scenario.mpa, scenario.mpa];
 
%     plot_local_reachable_sets(scenario.mpa, scenario.is_allow_non_convex)
end
