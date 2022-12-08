function scenario = circle_scenario(options)
% CIRCLE_SCENARIO   Constructor for scenario with vehicles circular arranged heading to the center of the circle.
    options.is_allow_non_convex = false;
    options.recursive_feasibility = true;
    
    if options.amount <= 2
        options.plot_limits = [-0.5,5;1.5,2.5];
    else
        options.plot_limits = [-0.5,5;-0.5,4.5];
    end
    scenario = Scenario();
    % read from optionos
    scenario.options = options; 

    radius = 2;
    nVeh = options.amount;
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

    scenario.options.scenario_name = sprintf('%i-circle', scenario.options.amount);

    scenario.model = BicycleModel(veh.Lf,veh.Lr);
    
    if options.isPB
        % undirected coupling adjacency is complete
        scenario.adjacency = ones(nVeh,nVeh);
        scenario.semi_adjacency = ones(nVeh,nVeh);
       if scenario.assignPrios
            scenario.directed_coupling = [];
       else
            scenario.directed_coupling = triu(ones(nVeh))-eye(nVeh);
       end
       scenario.controller_name = strcat(scenario.controller_name, '-PB');
       scenario.controller = @(s,i) pb_controller(s,i);
    else
        % no coupling?
        scenario.adjacency = zeros(nVeh,nVeh);
        scenario.directed_coupling = zeros(nVeh,nVeh);
        % not priority-based
        scenario.priority_list = ones(1,nVeh);
    end

    scenario.mpa = MotionPrimitiveAutomaton(scenario.model, options);
end