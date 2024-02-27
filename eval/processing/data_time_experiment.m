function computation_time = data_time_experiment(experiment_result)

    arguments (Input)
        experiment_result (1, 1) ExperimentResult
    end

    arguments (Output)
        % n_vehicles x n_steps
        computation_time (:, :) double
    end

    control_loop_timing = vertcat(experiment_result.timing.control_loop);
    control_loop_duration = control_loop_timing(2:2:end, :);

    synchronize_timing = vertcat(experiment_result.timing.receive_from_others);
    synchronize_duration = synchronize_timing(2:2:end, :);
    synchronize_min_duration = min(synchronize_duration);

    computation_time = control_loop_duration ...
        - synchronize_duration ...
        + repmat(synchronize_min_duration, size(control_loop_duration, 1), 1);
end
