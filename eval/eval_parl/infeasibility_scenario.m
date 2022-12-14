% prepare simulation options
options = OptionsMain;
options.consider_RSS = false;
options.is_sim_lab = true;
options.customResultName = '';
options.scenario_name = 'Commonroad';
options.trim_set = 9;
options.Hp = 4;

options.T_end = 1;
options.dt = 0.2;
options.max_num_CLs = inf;
options.priority = 'STAC_priority';
options.isPB = true;
options.isAllowInheritROW = false;
options.isSaveResult = false;
options.isSaveResultReduced = true;
options.visu = [true,false];
options.is_eval = false;
options.visualize_reachable_set = false;
options.strategy_consider_veh_without_ROW = '1';
options.strategy_enter_lanelet_crossing_area = '1';
options.is_force_parallel_vehs_in_same_grp = false;

options.amount = 3;
% options.veh_ids = [18,18,21];

options.reference_path.lanelets_index = {[69,64,62,75,74],[76,24,13,15],[95,69,64,62,75]};
options.reference_path.start_point = [1,4,7];

full_path = FileNameConstructor.get_results_full_path(options);
%     if isfile(full_path)
%         disp('File already exists.')
%     else
% run simulation
if exist('options','var') && exist('scenario','var')
    [~,~,~] = main(options,scenario);
else
    [~,scenario,~] = main(options);
end