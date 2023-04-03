function scenario = lanelet_scenario2(is_prioritized)
    % LANELET_SCENARIO2   Constructor for scenario with two vehicles at an intersection one driving left and one straight.

    c = 0.2;

    scenario = Scenario();
    [lanelets, collision] = intersection_lanelets();
    scenario.lanelets = cellfun(@(S) c * S(:, :), lanelets, 'Uniform', 0);

    scenario.vehicle_to_lanelet = [7, 12, 4; ...
                                       5, 19, 8];

    veh = Vehicle();
    veh.trim_config = 1;
    veh.x_start = c * -18;
    veh.y_start = c * -2.5;
    veh.yaw_start = 0;
    veh.x_goal = c * 18;
    veh.y_goal = c * -2.5;
    veh.yaw_goal = 0;
    veh_lanelets = vertcat(scenario.lanelets{scenario.vehicle_to_lanelet(1, :)});
    veh.reference_trajectory = unique([veh_lanelets(:, LaneletInfo.cx), veh_lanelets(:, LaneletInfo.cy)], 'rows', 'stable');
    scenario.vehicles = [scenario.vehicles, veh];

    veh = Vehicle();
    veh.trim_config = 1;
    veh.x_start = c * 2.5;
    veh.y_start = c * -18;
    veh.yaw_start = 0.5 * pi;
    veh.x_goal = c * -18;
    veh.y_goal = c * 2.5;
    veh.yaw_goal = pi;
    veh_lanelets = vertcat(scenario.lanelets{scenario.vehicle_to_lanelet(2, :)});
    veh.reference_trajectory = unique([veh_lanelets(:, LaneletInfo.cx), veh_lanelets(:, LaneletInfo.cy)], 'rows', 'stable');
    scenario.vehicles = [scenario.vehicles, veh];

    scenario.options.plot_limits = c * [-20, 20; -20, 20];
    scenario.name = sprintf('%i-intersection', scenario.options.amount);

    scenario.model = BicycleModel(veh.Lf, veh.Lr);

    nVeh_mpa = scenario.options.amount;

    if is_prioritized
        scenario.adjacency = coupling_adjacency_lanelets(scenario.vehicle_to_lanelet, collision);
        scenario.assignPrios = true;
        scenario.controller_name = strcat(scenario.controller_name, '-PB');
        scenario.controller = @(s, i) pb_controller(s, i);

        nVeh_mpa = 1;
    end

    scenario.mpa = MotionPrimitiveAutomaton(scenario.model, options);
end
