function experiment_results = collect_results_nuc(optional)

    arguments
        optional.options = Config.load_from_file('Config.json');
        optional.vehicle_ids (:, 1) double = 1:Config.load_from_file('Config.json').amount;
    end

    fprintf('Collecting experiment_results from remote HLCs...');

    vehicle_ids_arg = sprintf(' %d', optional.vehicle_ids);

    % stop session on all remote hlcs
    script_path = fullfile(pwd, 'nuc_control', 'stop_nuc.sh');
    command = ['bash ', script_path, vehicle_ids_arg];
    [~, ~] = system(command);

    % collect all ExperimentResults from nucs
    script_path = fullfile(pwd, 'nuc_control', 'collect_results_nuc.sh');
    command = ['bash ', script_path, vehicle_ids_arg];
    [~, ~] = system(command);

    fprintf(' done.\n')

    % load experiment_results

    experiment_results = cell(size(optional.vehicle_ids));

    % get list of all current experiment_results
    eval_files_folder = dir('/tmp/eval_files_*');
    current_eval_folder = eval_files_folder(end);
    current_eval_folder_dir = [current_eval_folder.folder, filesep, current_eval_folder.name];
    results_list = dir([current_eval_folder_dir, filesep, 'result_*']);

    % load and merge iteratively
    for i_entry = 1:numel(results_list)
        entry = results_list(i_entry);
        load([entry.folder, filesep, entry.name], 'experiment_result');
        experiment_results{i_entry} = experiment_result;
    end

end
