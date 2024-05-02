function computation_time = data_time_group_optimize_experiment(experiment_result)

    arguments (Input)
        experiment_result (1, 1) ExperimentResult
    end

    arguments (Output)
        % n_vehicles x n_steps
        computation_time (:, :) double
    end

    % assume all vehicles start at the same time with prioritization
    group_timing = vertcat(experiment_result.timing.group);
    group_duration = group_timing(2:2:end, :);

    computation_time = group_duration;

    computation_time = add_optimize_time(experiment_result, computation_time);

end
