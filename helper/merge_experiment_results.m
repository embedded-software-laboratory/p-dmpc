function experiment_result = merge_experiment_results(experiment_results)

    arguments
        experiment_results (1, :) ExperimentResult;
    end

    experiment_result = experiment_results(1);

    for i = 2:length(experiment_results)
        experiment_result = merge_two_experiment_results(experiment_result, experiment_results(i));
    end

end

function merged_experiment_result = merge_two_experiment_results(merged_experiment_result, result2)

    arguments
        merged_experiment_result (1, 1) ExperimentResult;
        result2 (1, 1) ExperimentResult;
    end

    i_veh = find(result2.n_expanded(:, 1) ~= 0);

    % if this is the first ExperimentResult just copy
    if isempty(merged_experiment_result)
        merged_experiment_result = result2;
        merged_experiment_result.total_fallback_times = zeros(result2.options.amount, 1);
        merged_experiment_result.total_fallback_times(i_veh) = result2.total_fallback_times;
        return;
    end

    merged_experiment_result.iteration_data = merge_two_iteration_data_objects(merged_experiment_result.iteration_data, result2.iteration_data);
    merged_experiment_result.trajectory_predictions(i_veh, :) = result2.trajectory_predictions(i_veh, :);

    % fallback ids are just locally available
    % therefore merge them as sets
    for i_step = 1:merged_experiment_result.n_steps
        merged_experiment_result.vehicles_fallback{i_step} = union(merged_experiment_result.vehicles_fallback{i_step}, result2.vehicles_fallback{i_step});
    end

    % deadlock as boolean
    % maybe store that beforehand for every vehicle-timestep-combination
    merged_experiment_result.n_expanded(i_veh, :) = result2.n_expanded(i_veh, :);
    % INFO: ignore all coupling info, they are the same on each nuc (dont know why they are safed in ExperimentResult AND in IterationData)

    % if someone needs one of these:
    % TODO: obstacles
    % TODO: lanelet_crossing_areas
    % TODO: all times need to be checked on how to compute when merged (are they even used anymore)
    merged_experiment_result.total_fallback_times(i_veh) = result2.total_fallback_times;

    % Timings
    merged_experiment_result.timing(i_veh) = result2.timing;
end

function iter = merge_two_iteration_data_objects(iter, other_iter)
    n_steps = numel(iter);
    i_veh = find(other_iter{1}.reference_trajectory_index(:, 1) ~= 0);

    % merge iteration struct for every timestep
    for i_step = 1:n_steps
        iter{i_step}.reference_trajectory_points(i_veh, :, :) = other_iter{i_step}.reference_trajectory_points(i_veh, :, :);
        iter{i_step}.reference_trajectory_index(i_veh, :, :) = other_iter{i_step}.reference_trajectory_index(i_veh, :, :);
        iter{i_step}.v_ref(i_veh, :) = other_iter{i_step}.v_ref(i_veh, :);
    end

end
