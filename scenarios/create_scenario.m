function scenario = create_scenario(options)
    %create scenario from options

    %% Setup Scenario
    switch options.scenario_type
        case ScenarioType.circle
            scenario = circle_scenario(options.amount);
        case ScenarioType.commonroad
            scenario = commonroad_scenario(options.amount, options.path_ids);
        case ScenarioType.lanelet2
            scenario = lanelet2_scenario(options.amount, options.path_ids, options.scenario_type);
        case ScenarioType.lab_default
            scenario = lanelet2_scenario(options.amount, options.path_ids, options.scenario_type);
    end

    % assign options to scenario object
    scenario.options = options;

end
