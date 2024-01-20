function experiment_result = merge_experiment_results(experiment_results)

    arguments
        experiment_results (1, :) ExperimentResult;
    end

    experiment_result = experiment_results(1);

    for i = 2:length(experiment_results)
        experiment_result = merge_two_experiment_results(experiment_result, experiment_results(i));
    end

    experiment_result.control_results_info = merge_control_results_info(experiment_results);

    % save the ExperimentResult to a file
    file_name = FileNameConstructor.get_results_full_path(experiment_result.options);
    save(file_name, 'experiment_result');
    fprintf('Merged results were saved in: %s\n', file_name);

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

    merged_experiment_result.iteration_data = merge_two_iteration_data_objects(merged_experiment_result.iteration_data, result2.iteration_data);

    % INFO: ignore all coupling info, they are the same on each nuc (dont know why they are saved in ExperimentResult AND in IterationData)

    % if someone needs one of these:
    % TODO: obstacles
    % TODO: lanelet_crossing_areas
    % TODO: all times need to be checked on how to compute when merged (are they even used anymore)

    % Timings
    merged_experiment_result.timing(i_veh) = result2.timing;
end

function iter = merge_two_iteration_data_objects(iter, other_iter)
    n_steps = numel(iter);
    i_veh = find(other_iter(1).reference_trajectory_index(:, 1) ~= 0);

    % merge iteration struct for every timestep
    for i_step = 1:n_steps
        iter(i_step).reference_trajectory_points(i_veh, :, :) = other_iter(i_step).reference_trajectory_points(i_veh, :, :);
        iter(i_step).reference_trajectory_index(i_veh, :, :) = other_iter(i_step).reference_trajectory_index(i_veh, :, :);
        iter(i_step).v_ref(i_veh, :) = other_iter(i_step).v_ref(i_veh, :);
    end

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
