%% evaluate parallel computation
customResultName = '';
scenario_name = 'Commonroad';
controller_name = 'RHC-Parl';
trim_set = 9;
Hp = 5;
dt = 0.3;
nVeh = 20;
T_end = 20;
priority_option = 'right_of_way_priority';
isParl = true;
isAllowInheritROW = true;
max_num_CLs = 8;
strategy_consider_veh_without_ROW = '3';
strategy_enter_lanelet_crossing_area = '4';
results_full_path = FileNameConstructor.get_results_full_path(customResultName,scenario_name,controller_name,trim_set,...
                Hp,dt,nVeh,T_end,priority_option,isParl,isAllowInheritROW,max_num_CLs,strategy_consider_veh_without_ROW,strategy_enter_lanelet_crossing_area);
% evaluation = EvaluationParl(results_full_path);
% disp(['Average run time: ' num2str(mean(evaluation.subcontroller_runtime_per_step)) ' seconds.'])
% disp(['Maximum 8 average run time: ' mat2str(round(maxk(evaluation.subcontroller_runtime_per_step,8),3)) ' seconds.'])
evaluation = EvaluationParl('test_a');
evaluation = EvaluationParl('boundaryCheckAfterImprovement');
% eval_before_2 = struct(evaluation_before);
disp(['Average run time: ' num2str(mean(evaluation.runtime_average)) ' seconds.'])
disp(['Maximum 8 average run time: ' mat2str(round(evaluation.runtime_max,3)) ' seconds.'])





%% bar chart: compare different maximum number of groups
max_num_CLs_all = 1:10;
evaluations = cell(1,length(max_num_CLs_all));
for i_CL = 1:length(max_num_CLs_all)
    customResultName = '';
    max_num_CLs = max_num_CLs_all(i_CL);
    scenario_name = 'Commonroad';
    controller_name = 'RHC-Parl';
    trim_set = 9;
    Hp = 5;
    dt = 0.2;
    nVeh = 20;
    T_end = 20;
    priority_option = 'right_of_way_priority';
    isParl = true;
    isAllowInheritROW = true;
    strategy_consider_veh_without_ROW = '3';
    strategy_enter_lanelet_crossing_area = '4';
    results_full_path = FileNameConstructor.get_results_full_path(customResultName,scenario_name,controller_name,trim_set,...
                    Hp,dt,nVeh,T_end,priority_option,isParl,isAllowInheritROW,max_num_CLs,strategy_consider_veh_without_ROW,strategy_enter_lanelet_crossing_area);
    evaluations{i_CL} = EvaluationParl(results_full_path);
    disp(i_CL)
%     disp(['Average run time: ' num2str(evaluations{i_CL}.runtime_average) ' seconds.'])
%     disp(['Maximum 8 average run time: ' mat2str(round(evaluations{i_CL}.runtime_max,3)) ' seconds.'])
end

% plot
runtime_average = cellfun(@(c) c.runtime_average, evaluations);
average_speed = cellfun(@(c) c.average_speed, evaluations);
fallback_rate = cellfun(@(c) c.fallback_rate*100, evaluations);

runtime_max = cellfun(@(c) c.runtime_max(1), evaluations);
runtime_max_plot = 0.32;
speed_max_plot = 0.8;
fallback_rate_max_plot = 5;

zoom_factors = [1,runtime_max_plot/speed_max_plot,runtime_max_plot/fallback_rate_max_plot];

data = [runtime_average;average_speed;fallback_rate]';
data_zoomed = [runtime_average*zoom_factors(1);average_speed*zoom_factors(2);fallback_rate*zoom_factors(3)]';
figure
catStrArray = {sprintf('Average runtime \\newline per step [s]'), ...
                sprintf('Average speed \\newline per step [m/s]'), ...
                sprintf('Fallback rate [%%]')};
catArray = categorical(catStrArray);       
% catArray = reordercats(catArray,catStrArray);
CM = jet(size(data,1));
b = bar(catArray,data_zoomed','FaceColor','flat');
nModel = size(data_zoomed,1);
nCat = size(data_zoomed,2);
xPosAmpl = 0.3682626-0.3298725*exp(-0.407004*(nModel-1)); % position amplitude
xPosInc = 2*xPosAmpl/(nModel-1);
modelNames = [];
x_speed = zeros(nModel,1);
for idxModel=1:nModel
    bar_xPos = 1:nCat;
    if nModel~=1
        bar_xPos = bar_xPos-xPosAmpl+(idxModel-1)*xPosInc;
    end
    x_speed(idxModel) = bar_xPos(1);
    text(bar_xPos,data_zoomed(idxModel,:),num2str(data(idxModel,:)',...
        '%.2f'),'VerticalAlignment','bottom','HorizontalAlignment','center','FontSize',9,'FontName','Times New Roman'); 
    modelNames{idxModel}=sprintf('Maximum allowed number of computation levels: %d',idxModel);
    b(idxModel).CData = repmat(CM(idxModel,:),nCat,1);
end
legend(modelNames,'FontSize',9)
ylim([0,runtime_max_plot])
set(gca,'YTick',[],'FontName','Times New Roman')
switch strategy_consider_veh_without_ROW
    case '1'
        title_ = 'Not consider vehicles with lower priorities';
    case '2'
        title_ = 'Consider current occupied sets of vehicles with lower priorities';
    case '3'
        title_ = 'Consider emergency braking maneuver of vehicles with lower priorities';
    case '4'
        title_ = 'Consider one-step-shifted privious trajectories of vehicles with lower priorities';
    case '5'
        title_ = 'Consider one-step reachable set of vehicles with lower priorities';
end
title(title_,'FontSize',9,'FontName','Times New Roman')

hold on
% plot max runtime
plot(x_speed,runtime_max,'-r*','LineWidth',1,'DisplayName','Maximum runtime per step [s]')
text(x_speed,runtime_max,num2str(runtime_max','%.2f'),'VerticalAlignment','bottom','HorizontalAlignment','center','FontSize',9,'FontName','Times New Roman'); 
%% Improve boundary check
e_before_improvement = EvaluationParl('boundaryCheckBeforeImprovement_ConsiderVehWithoutROW1');
e_after_improvement = EvaluationParl('boundaryCheckAfterImprovement_ConsiderVehWithoutROW1');
disp('--------Before improving boundary checking:')
disp(['Average run time: ' num2str(mean(e_before_improvement.runtime_average)) ' seconds.'])
disp(['Top 8 maximum runtime: ' mat2str(round(e_before_improvement.runtime_max,3)) ' seconds.'])
disp(['Average expanded nodes: ' num2str(e_before_improvement.nodes_expanded_average) '.'])
disp(['Average speed: ' num2str(e_before_improvement.average_speed) ' [m/s]'])

disp('--------After improving boundary checking:')
disp(['Average run time: ' num2str(mean(e_after_improvement.runtime_average)) ' seconds.'])
disp(['Top 8 maximum runtime: ' mat2str(round(e_after_improvement.runtime_max,3)) ' seconds.'])
disp(['Average expanded nodes: ' num2str(e_after_improvement.nodes_expanded_average) '.'])
disp(['Average speed: ' num2str(e_after_improvement.average_speed) ' [m/s]'])