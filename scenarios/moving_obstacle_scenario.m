function scenario = moving_obstacle_scenario(options)
    % MOVING_OBSTACLE_SCENARIO     Constructor for moving obstacle scenario
    arguments
        options.is_start_end (1, 1) logical = 0;
        options.is_plot_online (1, 1) logical = 1;
    end

    scenario = Scenario();

    veh = Vehicle();
    veh.x_start = 0;
    veh.y_start = 0;
    veh.yaw_start = 0;
    veh.x_goal = 100;
    veh.y_goal = 0;
    veh.yaw_goal = 0;
    veh.trim_config = 1;
    veh.referenceTrajectory = [veh.x_start veh.y_start; veh.x_goal veh.y_goal];

    scenario.vehicles = veh;
    scenario.options.veh_ids = 1;

    scenario.model = BicycleModel(veh.Lf, veh.Lr);

    scenario.options = OptionsMain;
    scenario.options.is_plot_online = options.is_plot_online;
    scenario.options.environment = Environment.Simulation;
    scenario.options.trim_set = 9;
    scenario.options.amount = 1;
    scenario.options.dt = 0.2;
    scenario.options.should_save_result = true;
    scenario.options.trim_set = 9;
    scenario.options.veh_ids = 1;
    scenario.options.T_end = 10;

    if ~options.is_start_end
        scenario.options.Hp = 5;
        scenario.options.scenario_name = sprintf('moving_obstacles');
    else
        scenario.options.Hp = round(scenario.options.T_end / scenario.options.dt);
        scenario.options.scenario_name = sprintf('moving_obstacles_start_end');
    end

    scenario.name = scenario.options.scenario_name;

    nVeh = 1;
    %scenario.adjacency = zeros(nVeh,nVeh);
    scenario.assignPrios = true;
    scenario.controller_name = strcat(scenario.controller_name, '-centralized');
    scenario.options.is_prioritized = false;
    scenario.controller = @centralized_controller;

    scenario.mpa = MotionPrimitiveAutomaton(scenario.model, scenario.options);

    %% Obstacles
    dx_veh_obs = 2.2;
    dx_obs_obs = 1.5;
    x_obs = [
             veh.x_start + dx_veh_obs;
             veh.x_start + dx_veh_obs + 1 * dx_obs_obs;
             veh.x_start + dx_veh_obs + 2 * dx_obs_obs;
             veh.x_start + dx_veh_obs + 3 * dx_obs_obs;
             ];

    y_obs_start = 4;
    y_obs = [
             y_obs_start + 0.1;
             -y_obs_start;
             y_obs_start;
             -y_obs_start;
             ];
    direction = [
                 1
                 -1
                 1
                 -1
                 ];

    dy_obs_obs = 1.7;
    speed_obs = 0.75;

    dist_dt = speed_obs * scenario.options.dt; % distance traveled per timestep
    width_obs = 0.5;
    length_obs = 0.5;
    length_coll_area = length_obs + dist_dt;

    scenario.dynamic_obstacle_shape = [width_obs; length_obs];

    n_obs_per_col = 10;
    n_cols = 2;

    for i_col = 1:n_cols

        for iObstacle = 1:n_obs_per_col

            for iTimestep = 1:(scenario.options.k_end + scenario.options.Hp + 1)
                scenario.dynamic_obstacle_area{iObstacle + (i_col - 1) * n_obs_per_col, iTimestep} = ...
                    transformedRectangle( ...
                    x_obs(i_col) ...
                    , y_obs(i_col) + direction(i_col) * (iTimestep * dist_dt - iObstacle * dy_obs_obs) ...
                    , pi / 2 ...
                    , length_coll_area ...
                    , width_obs ...
                );
                % Repeat first entry such that shape is closed to vectorize obstacles
                scenario.dynamic_obstacle_area{iObstacle + (i_col - 1) * n_obs_per_col, iTimestep}(:, end + 1) = ...
                    scenario.dynamic_obstacle_area{iObstacle + (i_col - 1) * n_obs_per_col, iTimestep}(:, 1);
                scenario.dynamic_obstacle_fullres{iObstacle + (i_col - 1) * n_obs_per_col, iTimestep} = ...
                    [ones(41, 1) * x_obs(i_col), ...
                     linspace(y_obs(i_col) + direction(i_col) * (iTimestep * dist_dt - iObstacle * dy_obs_obs - dist_dt / 2), ...
                     y_obs(i_col) + direction(i_col) * ((iTimestep + 1) * dist_dt - iObstacle * dy_obs_obs - dist_dt / 2), 41)'];
            end

        end

    end

    dx_plot = 0.5;
    scenario.options.plot_limits = [ ...
                                        veh.x_start - dx_plot, x_obs(n_cols) + dx_plot + dx_veh_obs; -1.5, 1.5 ...
                                    ];
end
