%% evaluate parallel computation
scenario_name = 'Commonroad';
controller_name = 'RHC-Parl';
trim_sest = 9;
Hp = 5;
dt = 0.2;
nVeh = 20;
T_end = 1000;
priority_option = 'right_of_way_priority';
isParl = true;
max_num_CLs = 2;
strategy_consider_veh_without_ROW = '1';
strategy_enter_lanelet_crossing_area = '4';
results_full_path = FileNameConstructor.get_results_full_path(scenario_name,controller_name,trim_sest,...
                Hp,dt,nVeh,T_end,priority_option,isParl,max_num_CLs,strategy_consider_veh_without_ROW,strategy_enter_lanelet_crossing_area);
% evaluation = EvaluationCommon(results_full_path);
% disp(['Average run time: ' num2str(mean(evaluation.subcontroller_runtime_per_step)) ' seconds.'])
% disp(['Maximum 8 average run time: ' mat2str(round(maxk(evaluation.subcontroller_runtime_per_step,8),3)) ' seconds.'])
evaluation = EvaluationCommon(results_full_path);
% eval_before_2 = struct(evaluation_before);
disp(['Average run time: ' num2str(mean(evaluation.runtime_average)) ' seconds.'])
disp(['Maximum 8 average run time: ' mat2str(round(evaluation.runtime_max,3)) ' seconds.'])

%% compare different maximum number of groups
max_num_CLs_all = 1:10;
evaluations = cell(1,length(max_num_CLs_all));
for i_CL = 1:length(max_num_CLs_all)
    max_num_CLs = max_num_CLs_all(i_CL);
    scenario_name = 'Commonroad';
    controller_name = 'RHC-Parl';
    trim_sest = 9;
    Hp = 5;
    dt = 0.2;
    nVeh = 20;
    T_end = 30;
    isParl = true;
%     max_num_CLs = 4;
    strategy_consider_veh_without_ROW = '3';
    strategy_enter_lanelet_crossing_area = '4';
    results_full_path = FileNameConstructor.get_results_full_path(scenario_name,controller_name,trim_sest,...
                    Hp,dt,nVeh,T_end,isParl,max_num_CLs,strategy_consider_veh_without_ROW,strategy_enter_lanelet_crossing_area);
    evaluations{i_CL} = EvaluationCommon(results_full_path);
    disp(['Average run time: ' num2str(evaluations{i_CL}.runtime_average) ' seconds.'])
    disp(['Maximum 8 average run time: ' mat2str(round(evaluations{i_CL}.runtime_max,3)) ' seconds.'])
end

%% plot
runtime_average = cellfun(@(c) c.runtime_average, evaluations);
average_speed = cellfun(@(c) c.average_speed, evaluations);
fallback_rate = cellfun(@(c) c.fallback_rate*100, evaluations);

runtime_max = cellfun(@(c) c.runtime_max(1), evaluations);

zoom_factors = [1,max(runtime_average)/max(average_speed),max(runtime_average)/max(fallback_rate)];

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

%% example
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
ylim([0,0.32])
set(gca,'YTick',[])

hold on
% plot max runtime
plot(x_speed,runtime_max,'-r*','LineWidth',1,'DisplayName','Maximum runtime per step [s]')
text(x_speed,runtime_max,num2str(runtime_max','%.2f'),'VerticalAlignment','bottom','HorizontalAlignment','center','FontSize',9,'FontName','Times New Roman'); 
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
