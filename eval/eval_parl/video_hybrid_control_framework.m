%% export video
%% two vehicles, parallel planning, visualize reachable sets
% prepare simulation options
options = Config;
options.environment = Environment.Simulation;
options.scenario_type = ScenarioType.commonroad;
options.mpa_type = MpaType.single_speed;
options.Hp = 6;

options.dt_seconds = 0.2;

options.priority = 'STAC_priority';
options.is_prioritized = true;
options.should_reduce_result = false;
options.is_plot_online = true;
options.constraint_from_successor = ConstraintFromSuccessor.area_of_standstill;

name = {'unboundedRSs', 'boundedRSs', 'laneletCrossingAreas', 'fullRun20Vehs30s', 'fullRun17VehsDeadlockRandomWeight'};
i = 1;
% options.result_name = name{i};
switch i
    case 1
        options.T_end = 3;
        options.options_plot_online.plot_reachable_sets = true;
        options.options_plot_online.plot_lanelet_crossing_areaas = false;
        options.options_plot_online.vehicles_reachable_sets = [2];
        options.is_bounded_reachable_set_used = false;
        options.constrained_enter_lanelet_crossing_area = false;
        options.max_num_CLs = 1;
        options.path_ids = [18, 41];
        options.amount = 2;
    case 2
        options.T_end = 3;
        options.options_plot_online.plot_reachable_sets = true;
        options.options_plot_online.plot_lanelet_crossing_areaas = false;
        options.is_bounded_reachable_set_used = true;
        options.constrained_enter_lanelet_crossing_area = false;
        options.max_num_CLs = 1;
        options.path_ids = [18, 41];
        options.amount = 2;
    case 3
        options.T_end = 3;
        options.options_plot_online.plot_reachable_sets = false;
        options.options_plot_online.plot_lanelet_crossing_areaas = true;
        options.is_bounded_reachable_set_used = true;
        options.constrained_enter_lanelet_crossing_area = true;
        options.max_num_CLs = 1;
        options.path_ids = [18, 41];
        options.amount = 2;
    case 4
        options.T_end = 30;
        options.options_plot_online.plot_reachable_sets = false;
        options.options_plot_online.plot_lanelet_crossing_areaas = false;
        options.is_bounded_reachable_set_used = true;
        options.constrained_enter_lanelet_crossing_area = true;
        options.max_num_CLs = 3;
        options.amount = 30;
        random_stream = RandStream('mt19937ar');
        options.path_ids = sort(randsample(random_stream, 1:40, options.amount), 'ascend');
    case 5
        options.T_end = 10;
        options.options_plot_online.plot_reachable_sets = false;
        options.options_plot_online.plot_lanelet_crossing_areaas = false;
        options.is_bounded_reachable_set_used = true;
        options.constrained_enter_lanelet_crossing_area = false;
        options.max_num_CLs = 4;
        options.amount = 17;
        options.priority = 'random_priority';
        random_stream = RandStream('mt19937ar');
        options.path_ids = sort(randsample(random_stream, 1:40, options.amount), 'ascend');
end

% options.options_plot_online.vehicles_reachable_sets = find(options.path_ids==41);
%
% options.options_plot_online.vehicles_lanelet_crossing_areas = find(options.path_ids==18);

if FileNameConstructor.result_exists(options)
    disp('File already exists.')
else
    % run simulation
    main(options);
end

experiment_result = load_latest(options);

%%
experiment_result.options.options_plot_online.is_video_mode = true;
experiment_result.options.options_plot_online.plot_coupling = true;
experiment_result.options.options_plot_online.plot_priority = false;

videoExportSetup.framerate = 5;
export_video(experiment_result, videoExportSetup)
