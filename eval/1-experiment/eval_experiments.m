function experiment_results = eval_experiments(optional)
    % EVAL_EXPERIMENTS Conduct experiments with a preset configuration for either
    % 1) different PriorityStrategies or
    % 2) different max_num_CLs

    arguments (Input)
        optional.scenario_type (1, 1) ScenarioType = ScenarioType.commonroad
        optional.computation_mode (1, 1) ComputationMode = ComputationMode.sequential
        optional.optimizer (1, 1) OptimizerType = OptimizerType.MatlabOptimal
        optional.priority_strategies (1, :) PriorityStrategies = PriorityStrategies.constant_priority
        optional.max_num_CLs (1, :) double = 99;
        optional.Hp (1, 1) double = 6;
    end

    arguments (Output)
        experiment_results (:, :, :) ExperimentResult % (n_vehicles x n_approaches x n_scenarios)
    end

    assert(isscalar(optional.max_num_CLs) ...
        || isscalar(optional.priority_strategies));

    options = Config();
    % Scenario
    options.scenario_type = optional.scenario_type;
    options.T_end = 7;
    % Scenario-specific config
    if options.scenario_type == ScenarioType.commonroad
        seeds = 1:3;
        n_vehicles_array = 5:5:20;
    elseif options.scenario_type == ScenarioType.circle
        seeds = 1;
        n_vehicles_array = 2:2:10;
    end

    % Environment
    options.options_plot_online.is_active = 0;
    options.computation_mode = optional.computation_mode;
    % High-Level Controller
    options.optimizer_type = optional.optimizer;
    options.mpa_type = MpaType.triple_speed;
    options.Hp = optional.Hp;

    experiment_results = ExperimentResult.empty();

    % experiment result in order (n_vehicles x n_approaches x n_scenarios)
    % number of different random scenarios per priority assignment and #vehicles
    for seed = seeds

        for n_vehicles = n_vehicles_array

            for priority = optional.priority_strategies
                options.priority = priority;

                for computation_levels = optional.max_num_CLs
                    options.max_num_CLs = computation_levels;

                    options.amount = n_vehicles;
                    options.path_ids = options.randomize_path_ids(seed = seed);

                    should_skip_commonroad = n_vehicles > 10 ...
                        && priority == PriorityStrategies.optimal_priority ...
                        && options.scenario_type == ScenarioType.commonroad;
                    should_skip_circle = n_vehicles > 5 ...
                        && priority == PriorityStrategies.optimal_priority ...
                        && options.scenario_type == ScenarioType.circle;

                    if should_skip_commonroad || should_skip_circle
                        experiment_result = ExperimentResult(options, -1);
                    else
                        experiment_result = FileNameConstructor.load_latest(options);

                        if isempty(experiment_result)
                            experiment_result = main(options);
                        end

                    end

                    experiment_results(end + 1) = experiment_result; %#ok<AGROW>
                end

            end

        end

    end

    experiment_results = reshape( ...
        experiment_results, ...
        max(length(optional.priority_strategies), length(optional.max_num_CLs)), ...
        length(n_vehicles_array), ...
        length(seeds) ...
    );
    experiment_results = permute(experiment_results, [2, 1, 3]);

end
