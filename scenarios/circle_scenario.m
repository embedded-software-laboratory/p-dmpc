function scenario = circle_scenario(nVeh,isPB)
% CIRCLE_SCENARIO   Constructor for scenario with vehicles circular arranged heading to the center of the circle.
    
    scenario = Scenario();
    
    radius = 2;
    yaws = pi*2/nVeh*(0:nVeh-1);
    for iVeh = 1:nVeh
        yaw = yaws(iVeh);
        s = sin(yaw);
        c = cos(yaw);
        veh = Vehicle();

        veh.trim_config = 1;

        veh.x_start = -c*radius;
        veh.y_start = -s*radius;
        veh.yaw_start = yaw;
        veh.x_goal = c*radius;
        veh.y_goal = s*radius;
        veh.yaw_goal = yaw;
        % Lab: translate by center
        center_x = 2.25;
        center_y = 2;
        veh.x_start = veh.x_start + center_x;
        veh.y_start = veh.y_start + center_y;
        veh.x_goal  = veh.x_goal  + center_x;
        veh.y_goal  = veh.y_goal  + center_y;

        veh.referenceTrajectory = [veh.x_start veh.y_start
                                   veh.x_goal  veh.y_goal];
        veh.ID = iVeh;

        scenario.vehicles = [scenario.vehicles, veh];
    end
    if nVeh <= 2
        scenario.plot_limits = [-0.5,5;1.5,2.5];
    else
        scenario.plot_limits = [-0.5,5;-0.5,4.5];
    end
    scenario.nVeh = nVeh;
    scenario.name = sprintf('%i-circle', scenario.nVeh);

    scenario.model = BicycleModel(veh.Lf,veh.Lr);

    scenario.T_end = 6;
   
    nVeh_mpa = scenario.nVeh;
    
    if isPB
        % undirected coupling adjacency is complete
        scenario.adjacency = ones(nVeh,nVeh);
       if scenario.assignPrios
            scenario.directed_coupling = [];
       else
            scenario.directed_coupling = triu(ones(nVeh))-eye(nVeh);
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