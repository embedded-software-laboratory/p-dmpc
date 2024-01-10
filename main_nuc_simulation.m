function experiment_results = main_nuc_simulation()
    close all
    clear all

    % read config from disk
    options = Config.load_from_file('Config.json');

    % read scenario from disk
    scenario = load('scenario.mat', 'scenario').scenario;

    vehicle_indices = 1:options.amount;

    fprintf('Starting remote HLCs...');
    script_path = fullfile(pwd, 'nuc_simulation', 'deploy_remote_hlcs.sh'); % assumes script is in curent directory

    command = ['bash ', script_path];

    for i_veh = vehicle_indices
        % hand over vehicle ids as arguments
        command = [command, ' ', num2str(i_veh)];
    end

    [~, ~] = system(command);
    fprintf(' done.\n')

    plotter = PlotterOnline(options, scenario);
    on_cleanup_function = onCleanup(@plotter.close_figure);

    if options.options_plot_online.is_active
        plotter.plotting_loop();
    else

        while ~plotter.is_finished
            pause(0.5);
        end

    end

    % stop session on all remote hlcs
    script_path = fullfile(pwd, 'nuc_simulation', 'stop_remote_hlcs.sh');

    command = ['bash ', script_path, ' ', num2str(numel(vehicle_indices))];

    [~, ~] = system(command);

    fprintf('Collecting experiment_results from remote HLCs...');

    % collect all ExperimentResults from nucs
    script_path = fullfile(pwd, 'nuc_simulation', 'collect_results.sh');

    command = ['bash ', script_path, ' ', num2str(numel(vehicle_indices))];

    [~, ~] = system(command);

    fprintf(' done.\n')

    % load & merge experiment_results
    fprintf('Merging experiment_results into one...');

    experiment_results = [];

    % get list of all current experiment_results
    eval_files_folder = dir('/tmp/eval_files_*');
    current_eval_folder = eval_files_folder(end);
    current_eval_folder_dir = [current_eval_folder.folder, filesep, current_eval_folder.name];
    results_list = dir([current_eval_folder_dir, filesep, 'veh_*']);

    % load and merge iteratively
    for i_entry = 1:numel(results_list)
        entry = results_list(i_entry);
        load([entry.folder, filesep, entry.name]);
        experiment_results = merge_experiment_results(experiment_results, experiment_result);
    end

    fprintf('done.\n');

end

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
    experiment_results.timings_per_vehicle(i_veh) = res.timings_per_vehicle(i_veh);
    experiment_results.timings_general(i_veh) = res.timings_general;
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
