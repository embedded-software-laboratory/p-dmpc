function scenario = create_scenario(options)
    %create scenario from options

    %% Setup Scenario
    switch options.scenario_type
        case ScenarioType.circle
            scenario = circle_scenario(options.amount);
        case ScenarioType.commonroad
            scenario = commonroad_scenario(options.amount, options.path_ids);
        case ScenarioType.lanelet2
            scenario = lanelet2_scenario(options.amount, options.path_ids);
        case ScenarioType.testbed_default
            scenario = Scenario();
            disp('Scenario is created later after map is retrieved via UnifiedTestbedInterface');

    end

end
