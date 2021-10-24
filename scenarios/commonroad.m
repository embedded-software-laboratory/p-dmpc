function scenario = commonroad(nVeh,vehid,isPB)
% Commonroad_Scenario   

    scenario = Scenario();
    scenario.name = 'Commonroad';
%     scenario.trim_set = 13;
%     scenario.dt = 0.2;
    scenario.trim_set = 12;
    scenario.dt = 0.4;
    scenario.commonroad = commonroad_lanelets();
    
    
    for iveh = 1:nVeh
        
        veh = Vehicle();
        veh.trim_config = 1;
        refPath = generate_ref_path(vehid(iveh));% function to generate refpath based on CPM Lab road geometry
        veh.x_start = refPath(1,1);
        veh.y_start = refPath(1,2);
        veh.x_goal = refPath(2:end,1);
        veh.y_goal = refPath(2:end,2);
        
        veh.referenceTrajectory = [veh.x_start veh.y_start
                                   veh.x_goal  veh.y_goal];

        yaw = calculate_yaw(vehid(iveh));
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
       scenario.adjacency = ones(nVeh,nVeh);
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


 


end
