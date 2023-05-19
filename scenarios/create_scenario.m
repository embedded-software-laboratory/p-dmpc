function scenario = create_scenario(options, random_seed, plant)
    %create scenario from options

    %% options preprocessing
    % Use options to setup scenario
    options.max_num_CLs = min(options.max_num_CLs, options.amount);

    if options.environment == Environment.Simulation
        disp('Running in MATLAB simulation...')

        if isempty(options.veh_ids)

            switch options.amount
                    % specify vehicles IDs
                case 2
                    vehicle_ids = [16, 18];
                case 4
                    vehicle_ids = [14, 16, 18, 20];
                    %       case 6
                    %           vehicle_ids = [10,14,16,17,18,20];
                otherwise
                    %           vehicle_ids = 1:options.amount; % default IDs
                    if options.max_num_CLs == 1
                        % if allowed computation is only 1, the first 8
                        % vehicles will not be used to avoid infeasibility at
                        % the first time step as there may be vehicles being
                        % very colse to others
                        vehicle_ids = sort(randsample(random_seed, 9:40, options.amount), 'ascend');
                    else
                        vehicle_ids = sort(randsample(random_seed, 1:40, options.amount), 'ascend');
                    end

            end

        else
            vehicle_ids = options.veh_ids;
        end

        options.veh_ids = sort(vehicle_ids);
        options.is_manual_control = 0;
    else
        disp('Running in CPM Lab or via Unified Lab API...')
        vehicle_ids = options.veh_ids;
        options.is_prioritized = true;

        options.veh_ids = sort(vehicle_ids);
    end

    assert(length(options.veh_ids) == options.amount);

    % more than 1 vehicle and use of pb-sequential controller
    % require out of process execution
    if options.amount > 1 && options.is_prioritized && ~options.compute_in_parallel
        options.mex_out_of_process_execution = true;
    end

    %% Setup Scenario
    switch options.scenario_name
        case ScenarioType.circle
            scenario = circle_scenario(options);
        case ScenarioType.commonroad
            scenario = commonroad(options, vehicle_ids);
        case 'Lanelet2'
            scenario = lanelet2_scenario(options, vehicle_ids, plant);
        case 'Lab_default'
            scenario = lanelet2_scenario(options, vehicle_ids, plant);
        case 'Lanelet2_IDS3C_circle'
            scenario = lanelet2_ids3c_circle_scenario(options, vehicle_ids, plant);
    end

    initial_state = find([scenario.mpa.trims.speed] == 0 & [scenario.mpa.trims.steering] == 0, 1);

    for iVeh = 1:options.amount
        % initialize vehicle ids of all vehicles
        scenario.vehicles(iVeh).ID = scenario.options.veh_ids(iVeh);
        scenario.vehicles(iVeh).trim_config = initial_state;

    end

    %plot_mpa_over_time(scenario);
    %plot_mpa(scenario,'do_export', true);
