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

    experiment_results = ExperimentResult.empty;

    % get list of all current experiment_results
    eval_files_folder = dir('/tmp/eval_files_*');
    current_eval_folder = eval_files_folder(end);
    current_eval_folder_dir = [current_eval_folder.folder, filesep, current_eval_folder.name];
    results_list = dir([current_eval_folder_dir, filesep, 'veh_*']);

    % load and merge iteratively
    for i_entry = 1:numel(results_list)
        entry = results_list(i_entry);
        load([entry.folder, filesep, entry.name]);
        experiment_results(end + 1) = experiment_result;
    end

    merge_experiment_results(experiment_results);

    fprintf('done.\n');

end
