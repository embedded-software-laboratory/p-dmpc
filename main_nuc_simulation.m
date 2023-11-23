function experiment_results = main_nuc_simulation()
    close all
    clear all

    % read config from disk
    options = Config.load_from_file('Config.json');

    % read scenario from disk
    scenario = load('scenario.mat', 'scenario').scenario;

    vehicle_ids = options.path_ids;

    fprintf('Starting remote HLCs...');
    script_path = fullfile(pwd, 'nuc_simulation', 'deploy_remote_hlcs.sh'); % assumes script is in curent directory

    command = ['bash ', script_path];

    for i_veh = vehicle_ids
        % hand over vehicle ids as arguments
        command = [command, ' ', num2str(i_veh)];
    end

    [~, ~] = system(command);
    fprintf(' done.\n')

    if options.options_plot_online.is_active
        plotter = PlotterOnline(options, scenario);
    end

    generate_plotting_info_msgs();
    ros2_node = ros2node('/plant_plotting');
    qos_config = struct("History", "keepall", "Reliability", "reliable", "Durability", "transientlocal");
    disp(['init subscriber for vehicles ', num2str(vehicle_ids)]);
    topic_name_subscribe = ['/plant_plotting'];
    subscriber = ros2subscriber(ros2_node, topic_name_subscribe, "plotting_info/PlottingInfo", @enqueue_plotting_info, qos_config);
    % initialize empty message queue
    global plotting_info_queue;
    plotting_info_queue = empty_plotting_info_queue();
    n_finished = 0;
    amount = options.amount;

    while true

        msg = dequeue_plotting_info();

        if ~isempty(msg)

            if msg.step == -1 % check whether end-message was sent
                n_finished = n_finished + 1;
            elseif options.options_plot_online.is_active
                plotter.ros2_callback(msg);
            end

        end

        if n_finished == amount % check if all vehicles have sent last time step
            disp('Reached end of simulation. Stop.')
            break
        end

        pause(0.001)

    end

    % stop session on all remote hlcs
    script_path = fullfile(pwd, 'nuc_simulation', 'stop_remote_hlcs.sh');

    command = ['bash ', script_path, ' ', num2str(numel(vehicle_ids))];

    [~, ~] = system(command);

    fprintf('Collecting experiment_results from remote HLCs...');

    % collect all ExperimentResults from nucs
    script_path = fullfile(pwd, 'nuc_simulation', 'collect_results.sh');

    command = ['bash ', script_path, ' ', num2str(numel(vehicle_ids))];

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
    experiment_results.vehicle_path_fullres(i_veh, :) = res.vehicle_path_fullres(i_veh, :);
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

function generate_plotting_info_msgs()
    % generate message needed for plotting via ros
    msgList = ros2("msg", "list"); % get all ROS 2 message types
    % check if message type is present
    if ((sum(cellfun(@(c)strcmp(c, 'plotting_info/PlottingInfo'), msgList)) == 0))
        [file_path, ~, ~] = fileparts(mfilename('fullpath'));
        disp('Generating ROS 2 custom message type for distributed plotting...')

        try
            ros2genmsg([file_path, filesep, 'plant']);
        catch ME
            disp(['If all environments for ros2genmsg() are prepared but still failed, try to move the whole folder to a ' ...
                  'shallower path and run again if you use Windows machine, which sadly has a max path limit constraint.'])
            throw(ME)
        end

    else
        disp(['No generation of ROS 2 custom message type for distributed plotting, since at least "plotting_info/PlottingInfo" ' ...
              'message exists. If message types are missing regenerate all by removing the folder matlab_msg_gen...'])
    end

end

function enqueue_plotting_info(msg)
    % add new message to end of queue
    global plotting_info_queue;
    plotting_info_queue(end + 1) = msg;
end

function msg = dequeue_plotting_info()
    global plotting_info_queue;

    if ~isempty(plotting_info_queue)
        % get and remove first message from queue
        msg = plotting_info_queue(1);
        plotting_info_queue(1) = [];
    else
        msg = [];
    end

end

function plotting_info_queue = empty_plotting_info_queue()
    plotting_info_queue = struct('trajectory_predictions', [], 'ref_trajectory', [], ...
        'n_obstacles', [], ...
        'n_dynamic_obstacles', [], 'step', [], ...
        'veh_indices', [], 'tick_now', [], ...
        'weighted_coupling_reduced', [], 'directed_coupling', [], ...
        'directed_coupling_sequential', [], ...
        'is_virtual_obstacle', [], ...
        'MessageType', []);
    % at this point plotting_info_queue is not an empty struct but one with empty entries
    % remove this entry for queue consistency reason
    plotting_info_queue(1) = [];
end
