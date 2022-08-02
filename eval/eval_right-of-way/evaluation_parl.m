%% evaluate parallel computation
scenario_name = 'Commonroad';
controller_name = 'RHC-Parl';
trim_sest = 9;
Hp = 5;
dt = 0.2;
nVeh = 20;
T_end = 300;
isParl = true;
max_num_CLs = 4;
strategy_consider_veh_without_ROW = '3';
strategy_enter_intersecting_area = '4';
results_full_path = FileNameConstructor.get_results_full_path(scenario_name,controller_name,trim_sest,...
                Hp,dt,nVeh,T_end,isParl,max_num_CLs,strategy_consider_veh_without_ROW,strategy_enter_intersecting_area);
% evaluation = EvaluationCommon(results_full_path);
% evaluation = evaluation.get_path_tracking_errors;
% evaluation = evaluation.get_total_runtime_per_step;
% evaluation = evaluation.get_average_speeds;
% disp(['Average run time: ' num2str(mean(evaluation.subcontroller_runtime_per_step)) ' seconds.'])
% disp(['Maximum 8 average run time: ' mat2str(round(maxk(evaluation.subcontroller_runtime_per_step,8),3)) ' seconds.'])
evaluation_before = EvaluationCommon('beforeDeleteReferencePoints_2.mat');
evaluation_before = evaluation_before.get_path_tracking_errors;
evaluation_before = evaluation_before.get_total_runtime_per_step;
evaluation_before = evaluation_before.get_average_speeds;
% eval_before_2 = struct(evaluation_before);
disp(['Average run time: ' num2str(mean(evaluation_before.subcontroller_runtime_per_step)) ' seconds.'])
disp(['Maximum 8 average run time: ' mat2str(round(maxk(evaluation_before.subcontroller_runtime_per_step,8),3)) ' seconds.'])
evaluation_after = EvaluationCommon('afterDeleteReferencePoints_3.mat');
evaluation_after = evaluation_after.get_path_tracking_errors;
evaluation_after = evaluation_after.get_total_runtime_per_step;
evaluation_after = evaluation_after.get_average_speeds;
disp(['Average run time: ' num2str(mean(evaluation_after.subcontroller_runtime_per_step)) ' seconds.'])
disp(['Maximum 8 average run time: ' mat2str(round(maxk(evaluation_after.subcontroller_runtime_per_step,8),3)) ' seconds.'])
% original trim primitives: D:\Documents\MATLAB\graph_based_planning\results\Commonroad_RHC-Parl\trims9_Hp5_dt0.2_nVeh20_T10_maxCLs4_ConsiderVehWithoutROW3_EnterIntersectingArea4.mat
% improved trim primitives: 
evaluation_block = EvaluationCommon('afterBlockVehicles_2.mat');
evaluation_block = evaluation_block.get_path_tracking_errors;
evaluation_block = evaluation_block.get_total_runtime_per_step;
evaluation_block = evaluation_block.get_average_speeds;
disp(['Average run time: ' num2str(mean(evaluation_block.subcontroller_runtime_per_step)) ' seconds.'])
disp(['Maximum 8 average run time: ' mat2str(round(maxk(evaluation_block.subcontroller_runtime_per_step,8),3)) ' seconds.'])