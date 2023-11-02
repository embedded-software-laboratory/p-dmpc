function scenario = create_scenario(options, plant)
    %create scenario from options

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

    % assign options to scenario object
    scenario.options = options;

end
