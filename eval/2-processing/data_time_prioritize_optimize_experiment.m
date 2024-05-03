function computation_time = data_time_prioritize_optimize_experiment(experiment_result)

    arguments (Input)
        experiment_result (1, 1) ExperimentResult
    end

    arguments (Output)
        % n_vehicles x n_steps
        computation_time (:, :) double
    end

    computation_time = zeros(experiment_result.n_hlc, experiment_result.n_steps);

    % assume all vehicles start at the same time with prioritization
    if experiment_result.options.priority ~= PriorityStrategies.optimal_priority
        prioritize_timing = vertcat(experiment_result.timing.prioritize);
        prioritize_duration = prioritize_timing(2:2:end, :);

        computation_time = computation_time + prioritize_duration;
    end

    computation_time = add_optimize_time(experiment_result, computation_time);

end
