function experiment_results = main_nuc(optional)

    arguments
        optional.vehicle_ids = [];
    end

    % On first experiment, push files to remote with `push_files_to_remote()`
    % On changes to ROS messages or MPA library, remove files with `remove_cash_remote()`

    % read config from disk
    options = Config.load_from_file('Config.json');

    % read scenario from disk
    scenario = load('scenario.mat', 'scenario').scenario;

    if options.environment == Environment.CpmLab
        assert(~isempty(optional.vehicle_ids), 'Vehicle ids are needed in CPM Lab');
    elseif options.environment == Environment.Simulation
        % vehicle ids are not needed in simulation
        optional.vehicle_ids = 1:options.amount;
    end

    vehicle_ids_arg = sprintf(' %d', optional.vehicle_ids);

    fprintf('Starting remote HLCs...');

    script_path = fullfile(pwd, 'nuc_simulation', 'deploy_remote_hlcs.sh');
    log_path = fullfile(pwd, 'nuc_simulation', 'deploy_remote_hlcs.log');
    command = ['bash ', script_path, vehicle_ids_arg, ' &> ', log_path];
    [~, ~] = system(command);

    fprintf(' done.\n')

    if options.options_plot_online.is_active
        plotter = PlotterOnline(options, scenario);
        on_cleanup_function = onCleanup(@plotter.close_figure);
        plotter.plotting_loop();
    else
        input('Press enter after experiment to collect results...');
    end

    % stop session on all remote hlcs
    script_path = fullfile(pwd, 'nuc_simulation', 'stop_remote_hlcs.sh');
    command = ['bash ', script_path, vehicle_ids_arg];
    [~, ~] = system(command);

    fprintf('Collecting experiment_results from remote HLCs...');

    % collect all ExperimentResults from nucs
    script_path = fullfile(pwd, 'nuc_simulation', 'collect_results.sh');
    command = ['bash ', script_path, vehicle_ids_arg];
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
        load([entry.folder, filesep, entry.name], 'experiment_result');
        experiment_results(end + 1) = experiment_result; %#ok<AGROW>
    end

    merge_experiment_results(experiment_results);

    fprintf('done.\n');

end
