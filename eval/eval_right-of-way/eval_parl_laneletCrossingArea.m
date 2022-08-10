%% evaluate parallel computation
customResultName = '';
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
max_num_CLs = 13;
strategy_consider_veh_without_ROW = '3';
strategy_enter_lanelet_crossing_area = '4';
results_full_path = FileNameConstructor.get_results_full_path(customResultName,scenario_name,controller_name,trim_set,...
                Hp,dt,nVeh,T_end,priority_option,isParl,isAllowInheritROW,max_num_CLs,strategy_consider_veh_without_ROW,strategy_enter_lanelet_crossing_area);
% evaluation = EvaluationCommon(results_full_path);
% disp(['Average run time: ' num2str(mean(evaluation.subcontroller_runtime_per_step)) ' seconds.'])
% disp(['Maximum 8 average run time: ' mat2str(round(maxk(evaluation.subcontroller_runtime_per_step,8),3)) ' seconds.'])
evaluation = EvaluationCommon(results_full_path);
% eval_before_2 = struct(evaluation_before);
disp(['Average run time: ' num2str(mean(evaluation.runtime_average)) ' seconds.'])
disp(['Maximum 8 average run time: ' mat2str(round(evaluation.runtime_max,3)) ' seconds.'])
%% Generate data: different maximum allowed number of computation level with not considering of lower-priority vehicles

load('optionsMain.mat','options')
options.customResultName = '';
options.scenario = 'Commonroad';
options.trim_set = 9;
options.Hp = 5;
options.dt = 0.2;
options.amount = 20;
options.T_end = 1000;
options.priority = 'right_of_way_priority';
options.isParl = true;
options.isAllowInheritROW = true;
options.max_num_CLs = 2;
options.strategy_consider_veh_without_ROW = '3';
options.strategy_enter_lanelet_crossing_area = '4';
options.isSaveResult = true;
options.visu = [true,false];
if exist('options','var') && exist('scenario','var')
    [~,scenario] = main(options,scenario);
else
    [~,scenario] = main(options);
end

disp('Done.')

%% plot: compare different maximum number of groups
max_num_CLs_all = 1:20;
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
    evaluations{i_CL} = EvaluationCommon(results_full_path);
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

% figure('Name','maxCLs')
% hold on
% 
% yyaxis left
% ylabel('Average runtime [s]')
% plot(max_num_CLs_all,runtime_average);
% ylim([0,0.2])
% 
% yyaxis right
% ylabel('Average speed [m/s]')
% plot(max_num_CLs_all,average_speed)
% ylim([0,0.8])
% 
% xlabel('Maximum allowed number of computation levels')
% title('Average runtime and traffic throughput')
% xticks(max_num_CLs_all)
% 
% grid on


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
%% Box plot
max_num_CLs_all = 1:20;
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
    evaluations{i_CL} = EvaluationCommon(results_full_path);
    disp(i_CL)
%     disp(['Average run time: ' num2str(evaluations{i_CL}.runtime_average) ' seconds.'])
%     disp(['Maximum 8 average run time: ' mat2str(round(evaluations{i_CL}.runtime_max,3)) ' seconds.'])
end
%%
set(0,'DefaultTextFontname', 'Times New Roman');
set(0,'DefaultAxesFontName', 'Times New Roman');
set(0,'defaultTextFontSize',9)
set(0,'defaultAxesFontSize',9)
label_options = {'FontSize',9,'FontName','Times New Roman','Interpreter','latex'};
runtime_s = cellfun(@(c) c.runtime_total_per_step, evaluations, 'UniformOutput',false);
runtime_average = cellfun(@(c) c.runtime_average, evaluations);
speed_s = cellfun(@(c) c.average_speed_each_veh, evaluations, 'UniformOutput',false);
fallback_rate = cellfun(@(c) c.fallback_rate*100, evaluations);
grp_runtime = cell2mat(arrayfun(@(i){i*ones(numel(runtime_s{i}),1)},(1:numel(runtime_s))')); 
grp_speed = cell2mat(arrayfun(@(i){i*ones(numel(speed_s{i}),1)},(1:numel(speed_s))')); 


fig = figure('Name','differentAllowedCLs');
fig.Position = [10 10 500 1600];
t_fig = tiledlayout(3,1,'TileSpacing','compact');
% boxplot runtime
% title(t_fig,'Compare different maximum allowed number of computation levels')
% subplot(3,1,1)
nexttile
grid on
hold on 
boxplot(vertcat(runtime_s{:}),grp_runtime,'Colors','k')
% plot line indicating the sample time 
p_dt = plot([0,length(runtime_s)+1],[dt,dt],'--b','LineWidth',1.0);
ylabel('Runtime per step $[s]$','Interpreter','latex')
ylim([0,0.32])
% boxplot speed
% subplot(3,1,2)
nexttile
grid on
hold on
boxplot(vertcat(speed_s{:}),grp_speed,'Colors','k')
ylabel('Average speed of each vehicle $[m/s]$','Interpreter','latex')
ylim([0.45,0.75])
% plot fallback rate
% subplot(3,1,3)
nexttile
grid on
hold on
plot(max_num_CLs_all,fallback_rate,'-*k','LineWidth',0.5)
ylabel('Fallback rate $\%$','Interpreter','latex')
legend([p_dt],{'Sample time $[s]$'},'Interpreter','latex')
xticks(max_num_CLs_all)

xlabel(t_fig,'Maximum allowed number of computation levels')

% save fig to pdf
set(fig,'Units','Inches');
pos = get(fig,'Position');
set(fig,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])

folder_target = fullfile(pwd,'fig');
if ~isfolder(folder_target)
    % create target folder if not exist
    mkdir(folder_target)
end

file_name = 'compareDifferentMaximumAllowedNumberOfCLs';

full_path = fullfile(folder_target,file_name);
if isfile(full_path)
    warning('The file for visualization of the offline reachable sets was already saved.');
else
    print(fig,full_path,'-dpdf','-r0');
    print(fig,full_path,'-dsvg','-r0');
end
%% example
err = [6, 3, 1; 1, 3, 4];
y=[69,123,137; 74, 62, 21];
figure
bar(y)
set(gca, 'XTickLabel', {'Cats' 'Dogs'});
hold on
errorbar(y, err, '.')
ylabel('Mean Error');
ylim([20 140]);
legend('response to novel cats', 'response to novel dogs');
saveas(gcf,['bar_2', '.jpg'])
%% Evaluate inherite right-of-way
