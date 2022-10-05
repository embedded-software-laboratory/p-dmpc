%% export video
%% two vehicles, parallel planning, visualize reachable sets
% prepare simulation options
options = OptionsMain;
options.consider_RSS = false;
options.is_sim_lab = true;
options.scenario_name = 'Commonroad';
options.trim_set = 9;
options.Hp = 6;


options.dt = 0.2;

options.priority = 'STAC_priority';
options.isPB = true;
options.isParl = true;
options.isAllowInheritROW = false;
options.isSaveResult = true;
options.isSaveResultReduced = false;
options.visu = [true,false];
options.is_eval = false;
options.visualize_reachable_set = false;
options.strategy_consider_veh_without_ROW = '3';


name = {'unboundedRSs','boundedRSs','laneletCrossingAreas','fullRun20Vehs30s','fullRun17VehsDeadlockRandomWeight'};
i = 1;
% options.customResultName = name{i};
switch i
    case 1
        options.T_end = 3;
        options.optionsPlotOnline.isShowReachableSets = true;
        options.optionsPlotOnline.isShowLaneletCrossingAreas = false;
        options.optionsPlotOnline.vehsReachableSets = [2];
        options.bound_reachable_sets = false;
        options.strategy_enter_lanelet_crossing_area = '1';
        options.max_num_CLs = 1;
        options.veh_ids = [18,41];
        options.amount = 2;
    case 2
        options.T_end = 3;
        options.optionsPlotOnline.isShowReachableSets = true;
        options.optionsPlotOnline.isShowLaneletCrossingAreas = false;
        options.bound_reachable_sets = true;
        options.strategy_enter_lanelet_crossing_area = '1';
        options.max_num_CLs = 1;
        options.veh_ids = [18,41];
        options.amount = 2;
    case 3
        options.T_end = 3;
        options.optionsPlotOnline.isShowReachableSets = false;
        options.optionsPlotOnline.isShowLaneletCrossingAreas = true;
        options.bound_reachable_sets = true;
        options.strategy_enter_lanelet_crossing_area = '4';
        options.max_num_CLs = 1;
        options.veh_ids = [18,41];
        options.amount = 2;
    case 4
        options.T_end = 30;
        options.optionsPlotOnline.isShowReachableSets = false;
        options.optionsPlotOnline.isShowLaneletCrossingAreas = false;
        options.bound_reachable_sets = true;
        options.strategy_enter_lanelet_crossing_area = '4';
        options.max_num_CLs = 3;
        options.amount = 30;
        random_seed = RandStream('mt19937ar');
        options.veh_ids = sort(randsample(random_seed,1:40,options.amount),'ascend');
    case 5
        options.T_end = 10;
        options.optionsPlotOnline.isShowReachableSets = false;
        options.optionsPlotOnline.isShowLaneletCrossingAreas = false;
        options.bound_reachable_sets = true;
        options.strategy_enter_lanelet_crossing_area = '1';
        options.max_num_CLs = 4;
        options.amount = 17;
        options.priority = 'random_priority';
        random_seed = RandStream('mt19937ar');
        options.veh_ids = sort(randsample(random_seed,1:40,options.amount),'ascend');
end



% options.optionsPlotOnline.vehsReachableSets = find(options.veh_ids==41);
% 
% options.optionsPlotOnline.vehsLaneletCorssingAreas = find(options.veh_ids==18);

full_path = FileNameConstructor.get_results_full_path(options);
if isfile(full_path)
    disp('File already exists.')
else
    % run simulation
    if exist('options','var') && exist('scenario','var')
        [~,~,~] = main(options,scenario);
    else
        [~,scenario,~] = main(options);
    end
end

load(full_path)

%%
result.scenario.options.optionsPlotOnline.isVideoMode = true;
result.scenario.options.optionsPlotOnline.isShowCoupling = true;
result.scenario.options.optionsPlotOnline.isShowPriority = false;

videoExportSetup.framerate = 5;
exportVideo(result,videoExportSetup)