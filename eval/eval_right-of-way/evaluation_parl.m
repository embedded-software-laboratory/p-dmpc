%% evaluate parallel computation
scenario_name = 'Commonroad';
controller_name = 'RHC-Parl';
trim_sest = 4;
Hp = 5;
nVeh = 20;
T_end = 20;
isParl = true;
max_num_CLs = 4;
strategy_consider_veh_without_ROW = '3';
strategy_enter_intersecting_area = '4';
results_full_path = FileNameConstructor.get_results_full_path(scenario_name,controller_name,trim_sest,...
                Hp,nVeh,T_end,isParl,max_num_CLs,strategy_consider_veh_without_ROW,strategy_enter_intersecting_area);
evaluation = EvaluationCommon(results_full_path);
evaluation = evaluation.get_path_tracking_errors;
evaluation = evaluation.get_total_runtime_per_step;