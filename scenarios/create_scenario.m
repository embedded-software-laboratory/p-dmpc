function scenario = create_scenario(options, plant)
    %create scenario from options

    %% options preprocessing
    % Use options to setup scenario

    if options.environment == Environment.Simulation || options.environment == Environment.SimulationDistributed
        disp('Running in MATLAB simulation...')
    else
        disp('Running in CPM Lab or via Unified Lab API...')
        options.is_prioritized = true;
    end

    % more than 1 vehicle and use of pb-sequential controller
    % require out of process execution
    if options.amount > 1 && options.is_prioritized && ~options.compute_in_parallel
        options.mex_out_of_process_execution = true;
    end

    %% Setup Scenario
    switch options.scenario_type
        case ScenarioType.circle
            scenario = circle_scenario(options);
        case ScenarioType.commonroad
            scenario = commonroad(options);
        case ScenarioType.lanelet2
            scenario = lanelet2_scenario(options, plant);
        case ScenarioType.lab_default
            scenario = lanelet2_scenario(options, plant);
    end

end
