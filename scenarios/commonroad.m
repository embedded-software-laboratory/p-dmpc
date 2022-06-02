function scenario = commonroad(options,vehid,mVehid,m2Vehid,is_sim_lab)
% Commonroad_Scenario   

    nVeh = options.amount;
    isPB = options.isPB;
    scenario = Scenario();
    scenario.name = 'Commonroad';
    scenario.trim_set = 4;
    scenario.dt = 0.2;
    scenario.options = options; 
    
    [scenario.lanelets,~, ~, scenario.intersection_lanelets, scenario.commonroad_data, scenario.lanelet_boundary] = commonroad_lanelets(options.mixedTrafficScenarioLanelets);
    
    for iveh = 1:nVeh
        
        veh = Vehicle();
        veh.trim_config = 1;

        if is_sim_lab
            [ref_path, scenario] = generate_random_path(scenario, vehid(iveh), 20, (vehid(iveh)+31));
        else
            if (mVehid == vehid(iveh) || m2Vehid == vehid(iveh))
                [ref_path, scenario] = generate_manual_path(scenario, vehid(iveh), 3, (vehid(iveh)+31), false);  
                %ref_path = generate_manual_path(scenario, vehid(iveh), 3, (vehid(iveh)));     
            else
                % ref_path = generate_ref_path(vehid(iveh));% function to generate refpath based on CPM Lab road geometry
                [ref_path, scenario] = generate_random_path(scenario, vehid(iveh), 3, (vehid(iveh)+31)); % function to generate random path for autonomous vehicles based on CPM Lab road geometry
                %ref_path = generate_random_path(scenario, vehid(iveh), 3, (vehid(iveh)));
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
    scenario.T_end = 60;
    scenario.model = BicycleModel(veh.Lf,veh.Lr);
    nVeh_mpa = scenario.nVeh;
    scenario.Hp = 6;
    
    if isPB 
        scenario.adjacency = zeros(nVeh,nVeh);
        scenario.assignPrios = true;
        scenario.controller_name = strcat(scenario.controller_name, '-PB');
        scenario.controller = @(s,i) pb_controller(s,i);
        nVeh_mpa = 1;

    end
%     
    
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

    % initialize speed profile vector, currently 3 speed profiles are available
    scenario.speed_profile_mpas = [scenario.mpa, scenario.mpa, scenario.mpa];
 


end
