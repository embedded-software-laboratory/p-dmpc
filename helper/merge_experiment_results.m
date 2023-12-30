function experiment_results = merge_experiment_results(experiment_results, res)

    i_veh = find(res.n_expanded(:, 1) ~= 0);

    % if this is the first ExperimentResult just copy
    if isempty(experiment_results)
        experiment_results = res;
        experiment_results.total_fallback_times = zeros(res.options.amount, 1);
        experiment_results.total_fallback_times(i_veh) = res.total_fallback_times;
        return;
    end

    experiment_results.iteration_data = merge_iteration_data(experiment_results.iteration_data, res.iteration_data);
    experiment_results.trajectory_predictions(i_veh, :) = res.trajectory_predictions(i_veh, :);

    % fallback ids are just locally available
    % therefore merge them as sets
    for i_step = 1:experiment_results.n_steps
        experiment_results.vehicles_fallback{i_step} = union(experiment_results.vehicles_fallback{i_step}, res.vehicles_fallback{i_step});
    end

    % deadlock as boolean
    % maybe store that beforehand for every vehicle-timestep-combination
    experiment_results.n_expanded(i_veh, :) = res.n_expanded(i_veh, :);
    % INFO: ignore all coupling info, they are the same on each nuc (dont know why they are safed in ExperimentResult AND in IterationData)

    % if someone needs one of these:
    % TODO: obstacles
    % TODO: lanelet_crossing_areas
    % TODO: all times need to be checked on how to compute when merged (are they even used anymore)
    experiment_results.total_fallback_times(i_veh) = res.total_fallback_times;

    % Timings
    experiment_results.timing(i_veh) = res.timing;
end

function iter = merge_iteration_data(iter, iter_in)
    n_steps = numel(iter);
    i_veh = find(iter_in{1}.reference_trajectory_index(:, 1) ~= 0);

    % merge iteration struct for every timestep
    for i_step = 1:n_steps
        iter{i_step}.reference_trajectory_points(i_veh, :, :) = iter_in{i_step}.reference_trajectory_points(i_veh, :, :);
        iter{i_step}.reference_trajectory_index(i_veh, :, :) = iter_in{i_step}.reference_trajectory_index(i_veh, :, :);
        iter{i_step}.v_ref(i_veh, :) = iter_in{i_step}.v_ref(i_veh, :);
    end

end
