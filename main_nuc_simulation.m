function results = main_nuc_simulation()
    close all
    clear all
    scenario = load('scenario.mat', 'scenario').scenario;
    veh_ids = scenario.options.veh_ids;

    fprintf('Starting remote HLCs...');
    script_path = fullfile(pwd, 'nuc_simulation', 'deploy_remote_hlcs.sh'); % assumes script is in curent directory

    command = ['bash ', script_path];

    for i_veh = veh_ids
        % hand over vehicle ids as arguments
        command = [command, ' ', num2str(i_veh)];
    end

    [~, ~] = system(command);
    fprintf(' done.\n')

    if scenario.options.options_plot_online.is_active
        plotter = PlotterOnline(scenario);
    end

    generate_plotting_info_msgs();
    ros2_node = ros2node('/plant_plotting');
    options = struct("History", "keepall", "Reliability", "reliable", "Durability", "transientlocal");
    disp(['init subscriber for vehicles ', num2str(veh_ids)]);
    topic_name_subscribe = ['/plant_plotting'];
    subscriber = ros2subscriber(ros2_node, topic_name_subscribe, "plotting_info/PlottingInfo", @enqueue_plotting_info, options);
    % initialize empty message queue
    global plotting_info_queue;
    plotting_info_queue = empty_plotting_info_queue();
    n_finished = 0;
    amount = scenario.options.amount;

    while true

        msg = dequeue_plotting_info();

        if ~isempty(msg)

            if msg.step == -1 % check whether end-message was sent
                n_finished = n_finished + 1;
            elseif scenario.options.options_plot_online.is_active
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

    command = ['bash ', script_path, ' ', num2str(numel(veh_ids))];

    [~, ~] = system(command);

    fprintf('Collecting results from remote HLCs...');

    % collect all result structs from nucs
    script_path = fullfile(pwd, 'nuc_simulation', 'collect_results.sh');

    command = ['bash ', script_path, ' ', num2str(numel(veh_ids))];

    [~, ~] = system(command);

    fprintf(' done.\n')

    % load & merge results
    fprintf('Merging results into one...');

    results = [];

    % get list of all current results
    eval_files_folder = dir('/tmp/eval_files_*');
    current_eval_folder = eval_files_folder(end);
    current_eval_folder_dir = [current_eval_folder.folder, filesep, current_eval_folder.name];
    results_list = dir([current_eval_folder_dir, filesep, 'veh_*']);

    % load and merge iteratively
    for i_entry = 1:numel(results_list)
        entry = results_list(i_entry);
        load([entry.folder, filesep, entry.name]);
        results = merge_results(results, result);
    end

    fprintf('done.\n');

end

function results = merge_results(results, res)

    i_veh = find(res.n_expanded(:, 1) ~= 0);

    % if this is the first result just copy
    if isempty(results)
        results = res;
        results.total_fallback_times = zeros(res.scenario.options.amount, 1);
        results.total_fallback_times(i_veh) = res.total_fallback_times;
        return;
    end

    results.iteration_structs = merge_iteration_structs(results.iteration_structs, res.iteration_structs);
    results.vehicle_path_fullres(i_veh, :) = res.vehicle_path_fullres(i_veh, :);
    results.trajectory_predictions(i_veh, :) = res.trajectory_predictions(i_veh, :);

    % fallback ids are just locally available
    % therefore merge them as sets
    for i_step = 1:results.nSteps
        results.vehs_fallback{i_step} = union(results.vehs_fallback{i_step}, res.vehs_fallback{i_step});
    end

    % deadlock as boolean
    % maybe store that beforehand for every vehicle-timestep-combination
    results.is_deadlock = results.is_deadlock | res.is_deadlock;
    results.subcontroller_runtime_each_veh(i_veh, :) = res.subcontroller_runtime_each_veh(i_veh, :);
    results.graph_search_runtime_each_veh(i_veh, :) = res.graph_search_runtime_each_veh(i_veh, :);
    results.n_expanded(i_veh, :) = res.n_expanded(i_veh, :);
    % INFO: ignore all coupling info, they are the same on each nuc (dont know why they are safed in result AND in IterationData)

    % if someone needs one of these:
    % TODO: obstacles
    % TODO: lanelet_crossing_areas
    % TODO: runtimes
    % TODO: all times need to be checked on how to compute when merged (are they even used anymore)
    results.total_fallback_times(i_veh) = res.total_fallback_times;

end

function iter = merge_iteration_structs(iter, iter_in)
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
        'priorities', [], 'n_obstacles', [], ...
        'n_dynamic_obstacles', [], 'step', [], ...
        'veh_indices', [], 'tick_now', [], ...
        'weighted_coupling_reduced', [], 'directed_coupling', [], ...
        'belonging_vector', [], 'is_virtual_obstacle', [], ...
        'MessageType', []);
    % at this point plotting_info_queue is not an empty struct but one with empty entries
    % remove this entry for queue consistency reason
    plotting_info_queue(1) = [];
end
