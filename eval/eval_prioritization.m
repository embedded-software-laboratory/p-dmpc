function eval_prioritization(optional)

    arguments
        optional.computation_mode (1, 1) ComputationMode = ComputationMode.sequential
    end

    priority_strategies = [
                           PriorityStrategies.constant_priority
                           PriorityStrategies.random_priority
                           PriorityStrategies.coloring_priority_priority
                           PriorityStrategies.FCA_priority_priority
                           PriorityStrategies.explorative_priority
                           PriorityStrategies.optimal_priority
                           ];

    scenarios = [ScenarioType.commonroad, ScenarioType.circle];
    optimizers = [OptimizerType.MatlabOptimal, OptimizerType.MatlabSampled];

    for scenario = scenarios

        for optimizer = optimizers
            % Generate results
            experiment_results = eval_experiments( ...
                computation_mode = optional.computation_mode, ...
                scenario_type = scenario, ...
                optimizer = optimizer, ...
                priority_strategies = priority_strategies ...
            );

            % Plot cost

            % Plot computation time
        end

    end

end
