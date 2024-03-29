%% export video
%% two vehicles, parallel planning, visualize reachable sets
% prepare simulation options
options = OptionsMain;
options.environment = Environment.Simulation;
options.scenario_name = ScenarioType.commonroad;
options.trim_set = 9;
options.Hp = 6;

options.dt = 0.2;

options.priority = 'STAC_priority';
options.is_prioritized = true;
options.allow_priority_inheritance = false;
options.should_save_result = true;
options.should_reduce_result = false;
options.is_plot_online = true;
options.is_eval = false;
options.strategy_consider_veh_without_ROW = '3';

name = {'unboundedRSs', 'boundedRSs', 'laneletCrossingAreas', 'fullRun20Vehs30s', 'fullRun17VehsDeadlockRandomWeight'};
i = 1;
% options.result_name = name{i};
switch i
    case 1
        options.T_end = 3;
        options.options_plot_online.plot_reachable_sets = true;
        options.options_plot_online.plot_lanelet_crossing_areaas = false;
        options.options_plot_online.vehicles_reachable_sets = [2];
        options.bound_reachable_sets = false;
        options.strategy_enter_lanelet_crossing_area = '1';
        options.max_num_CLs = 1;
        options.veh_ids = [18, 41];
        options.amount = 2;
    case 2
        options.T_end = 3;
        options.options_plot_online.plot_reachable_sets = true;
        options.options_plot_online.plot_lanelet_crossing_areaas = false;
        options.bound_reachable_sets = true;
        options.strategy_enter_lanelet_crossing_area = '1';
        options.max_num_CLs = 1;
        options.veh_ids = [18, 41];
        options.amount = 2;
    case 3
        options.T_end = 3;
        options.options_plot_online.plot_reachable_sets = false;
        options.options_plot_online.plot_lanelet_crossing_areaas = true;
        options.bound_reachable_sets = true;
        options.strategy_enter_lanelet_crossing_area = '4';
        options.max_num_CLs = 1;
        options.veh_ids = [18, 41];
        options.amount = 2;
    case 4
        options.T_end = 30;
        options.options_plot_online.plot_reachable_sets = false;
        options.options_plot_online.plot_lanelet_crossing_areaas = false;
        options.bound_reachable_sets = true;
        options.strategy_enter_lanelet_crossing_area = '4';
        options.max_num_CLs = 3;
        options.amount = 30;
        random_stream = RandStream('mt19937ar');
        options.veh_ids = sort(randsample(random_stream, 1:40, options.amount), 'ascend');
    case 5
        options.T_end = 10;
        options.options_plot_online.plot_reachable_sets = false;
        options.options_plot_online.plot_lanelet_crossing_areaas = false;
        options.bound_reachable_sets = true;
        options.strategy_enter_lanelet_crossing_area = '1';
        options.max_num_CLs = 4;
        options.amount = 17;
        options.priority = 'random_priority';
        random_stream = RandStream('mt19937ar');
        options.veh_ids = sort(randsample(random_stream, 1:40, options.amount), 'ascend');
end

% options.options_plot_online.vehicles_reachable_sets = find(options.veh_ids==41);
%
% options.options_plot_online.vehicles_lanelet_crossing_areas = find(options.veh_ids==18);

full_path = FileNameConstructor.get_results_full_path(options);

if isfile(full_path)
    disp('File already exists.')
else
    % run simulation
    if exist('options', 'var') && exist('scenario', 'var')
        [~, ~, ~] = main(options, scenario);
    else
        [~, scenario, ~] = main(options);
    end

end

load(full_path)

%%
result.scenario.options.options_plot_online.is_video_mode = true;
result.scenario.options.options_plot_online.plot_coupling = true;
result.scenario.options.options_plot_online.plot_priority = false;

videoExportSetup.framerate = 5;
export_video(result, videoExportSetup)
