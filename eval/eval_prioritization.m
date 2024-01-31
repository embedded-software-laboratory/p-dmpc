function results = eval_prioritization(optional)

    arguments
        optional.computation_mode (1, 1) ComputationMode = ComputationMode.sequential
    end

    priority_strategies = {
                           PriorityStrategies.constant_priority
                           PriorityStrategies.random_priority
                           PriorityStrategies.coloring_priority_priority
                           PriorityStrategies.FCA_priority_priority
                           PriorityStrategies.optimal_priority
                           PriorityStrategies.explorative_priority
                           };

    options = Config;
    options.computation_mode = optional.computation_mode;

    nsVeh = 1:20;
    % number of different random scenarios per priority assignment and #vehicles
    seeds = 1:9;

    scenarios = randomize_commonroad(options, nsVeh, seeds);

end
