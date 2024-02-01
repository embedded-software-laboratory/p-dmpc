function eval_grouping(optional)

    arguments
        optional.computation_mode (1, 1) ComputationMode = ComputationMode.sequential
    end

    max_num_CLs = [1, 2, 4, 6, 8, 99];

    scenarios = [ScenarioType.commonroad, ScenarioType.circle];
    optimizers = [OptimizerType.MatlabOptimal, OptimizerType.MatlabSampled];

    for scenario = scenarios

        for optimizer = optimizers
            % Generate results
            experiment_results = eval_experiments( ...
                computation_mode = optional.computation_mode, ...
                scenario_type = scenario, ...
                optimizer = optimizer, ...
                max_num_CLs = max_num_CLs ...
            );

            % Plot cost

            % Plot computation time
        end

    end

end
