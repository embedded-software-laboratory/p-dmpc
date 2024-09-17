function experiment_result = merge_experiment_results(experiment_results)

    arguments
        experiment_results (1, :) ExperimentResult;
    end

    experiment_result = experiment_results(1);

    for i = 2:length(experiment_results)
        experiment_result = merge_two_experiment_results(experiment_result, experiment_results(i));
    end

    experiment_result.control_results_info = merge_control_results_info(experiment_results);

    experiment_result = normalize_timing_results(experiment_result);

end

function merged_experiment_result = merge_two_experiment_results(merged_experiment_result, result2)

    arguments
        merged_experiment_result (1, 1) ExperimentResult;
        result2 (1, 1) ExperimentResult;
    end

    i_veh = result2.hlc_indices(1); % should be scalar, for safety reason

    % if this is the first ExperimentResult just copy
    if isempty(merged_experiment_result)
        merged_experiment_result = result2;
        return;
    end

    % merge indices of controlled hlcs in experiment result
    merged_experiment_result.hlc_indices = union(merged_experiment_result.hlc_indices, result2.hlc_indices);

    % INFO: ignore all coupling info, they are the same on each nuc (dont know why they are saved in ExperimentResult AND in IterationData)

    % if someone needs one of these:
    % TODO: obstacles
    % TODO: all times need to be checked on how to compute when merged (are they even used anymore)

    % Timings
    merged_experiment_result.timing(i_veh) = result2.timing;
end

function merged_control_results_info = merge_control_results_info(experiment_results)

    arguments (Input)
        experiment_results (1, :) ExperimentResult
    end

    arguments (Output)
        merged_control_results_info (:, :) ControlResultsInfo % n_vehicles x n_steps
    end

    merged_control_results_info = vertcat(experiment_results.control_results_info);

end
