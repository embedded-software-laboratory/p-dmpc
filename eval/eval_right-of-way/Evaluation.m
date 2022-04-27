% Evaluation   evaluate the right-of-way assignment method and the
% distributed controller


%% computation levels
load('graph_based_planning/eval/eval_right-of-way/results/result_02.mat')
computation_levels = result_02.computation_levels;
unique_computation_levels = unique(computation_levels);
n_unique_computation_levels = zeros(1,length(unique_computation_levels));

for i = 1:length(unique_computation_levels)
    n_unique_computation_levels(i) = sum(computation_levels == unique_computation_levels(i));
   
end

levels=bar(unique_computation_levels,n_unique_computation_levels);
text(unique_computation_levels,n_unique_computation_levels',num2str(n_unique_computation_levels'),'HorizontalAlignment','center','VerticalAlignment','bottom','FontSize', 24);
title('Distribution of Computation Levels')
xlabel('Computation Level')
ylabel('Number of Computation Levels')
set(gca,'FontSize',24)

%% controller time
% distributed controller
load('graph_based_planning/eval/eval_right-of-way/results/result_02.mat')
controller_runtime = result_02.controller_runtime;
max_runtime = max(controller_runtime);
min_runtime = min(controller_runtime);
avg_runtime = mean(controller_runtime);

X = categorical({'max\_runtime','avg\_runtime'});
X = reordercats(X,{'max\_runtime','avg\_runtime'});
Y = [max_runtime,avg_runtime];
runtime = bar(X,Y);
text(X,Y,num2str(Y','%20.4f'),'HorizontalAlignment','center','VerticalAlignment','bottom','FontSize', 24);
title('Distributed Controller Runtime','FontSize', 24)
ylim([0,0.8])
% xlabel('FontSize', 24)
ylabel('Time','FontSize', 24)
set(gca,'FontSize',24) 

% centralized controller
load('central_1.mat');
max_controller_time1 = max(result.controller_runtime);
load('central_2.mat');
max_controller_time2 = max(result.controller_runtime);
load('central_3.mat');
max_controller_time3 = max(result.controller_runtime);

figure
X = categorical({'1 vehicle','2 vehicles','3 vehicles'});
X = reordercats(X,{'1 vehicle','2 vehicles','3 vehicles'});
Y = [max_controller_time1,max_controller_time2,max_controller_time3];
priority = bar(X,Y);
text(X,Y,num2str(Y','%20.4f'),'HorizontalAlignment','center','VerticalAlignment','bottom','FontSize', 24);
title('Centralized-controller runtime','FontSize', 24)
ylim([0,100])
% xlabel('Types','FontSize', 24)
% ylabel('Time [s]','FontSize', 24)
set(gca,'FontSize',24)

%% deviations
% deviation 0.2s for single vehicle
% different trajectories
load('graph_based_planning/eval/eval_right-of-way/results/single.mat');
[deviation_single,max_deviation_single,avg_deviation_single, avg_vRef_single, avg_speed_single] = plot_deviation(result);
load('graph_based_planning/eval/eval_right-of-way/results/single6.mat');
[deviation_single6,max_deviation_single6,avg_deviation_single6, avg_vRef_single6, avg_speed_single6] = plot_deviation(result);
load('graph_based_planning/eval/eval_right-of-way/results/single10.mat');
[deviation_single10,max_deviation_single10,avg_deviation_single10, avg_vRef_single10, avg_speed_single10] = plot_deviation(result);
load('graph_based_planning/eval/eval_right-of-way/results/single11.mat');
[deviation_single11,max_deviation_single11,avg_deviation_single11, avg_vRef_single11, avg_speed_single11] = plot_deviation(result);
load('graph_based_planning/eval/eval_right-of-way/results/single15.mat');
[deviation_single15,max_deviation_single15,avg_deviation_single15, avg_vRef_single15, avg_speed_single15] = plot_deviation(result);
load('graph_based_planning/eval/eval_right-of-way/results/single16.mat');
[deviation_single16,max_deviation_single16,avg_deviation_single16, avg_vRef_single16, avg_speed_single16] = plot_deviation(result);
load('graph_based_planning/eval/eval_right-of-way/results/single19.mat');
[deviation_single19,max_deviation_single19,avg_deviation_single19, avg_vRef_single19, avg_speed_single19] = plot_deviation(result);
load('graph_based_planning/eval/eval_right-of-way/results/single21_02s.mat');
[deviation_single21,max_deviation_single21,avg_deviation_single21, avg_vRef_single21, avg_speed_single21] = plot_deviation(result);
max_deviation  = max([max_deviation_single,max_deviation_single6,max_deviation_single10,max_deviation_single11,...
    max_deviation_single15,max_deviation_single16,max_deviation_single19,max_deviation_single21]);
avg_deviation = mean([avg_deviation_single,avg_deviation_single6,avg_deviation_single10,avg_deviation_single11,...
    avg_deviation_single15,avg_deviation_single16,avg_deviation_single19,avg_deviation_single21]);

subplot(1,2,2)
hold on

X = categorical({'Max deviation','Average Deviation'});
X = reordercats(X,{'Max deviation','Average Deviation'});
Y = [max_deviation,avg_deviation];

deviations = bar(X,Y);
text(X,Y,num2str(Y','%20.4f'),'HorizontalAlignment','center','VerticalAlignment','bottom','FontSize', 24);
title('Max and Average Deviation ','FontSize', 24)
ylim([0,0.02])
ylabel('Deviation[m]','FontSize', 24)
set(gca,'FontSize',24)

% differen trim duration
load('graph_based_planning/eval/eval_right-of-way/results/single21_01s.mat');
[deviation_single21_01,max_deviation_single21_01,avg_deviation_single21_01, avg_vRef_single21_01, avg_speed_single21_01] = plot_deviation(result);
load('graph_based_planning/eval/eval_right-of-way/results/single21_02s.mat');
[deviation_single21_02,max_deviation_single21_02,avg_deviation_single21_02, avg_vRef_single21_02, avg_speed_single21_02] = plot_deviation(result);
load('graph_based_planning/eval/eval_right-of-way/results/single21_03s.mat');
[deviation_single21_03,max_deviation_single21_03,avg_deviation_single21_03, avg_vRef_single21_03, avg_speed_single21_03] = plot_deviation(result);
load('graph_based_planning/eval/eval_right-of-way/results/single21_04s.mat');
[deviation_single21_04,max_deviation_single21_04,avg_deviation_single21_04, avg_vRef_single21_04, avg_speed_single21_04] = plot_deviation(result);
figure
X = categorical({'Max deviation','Average Deviation'});
X = reordercats(X,{'Max deviation','Average Deviation'});
Y = [max_deviation_single21_01,avg_deviation_single21_01;...
    max_deviation_single21_02,avg_deviation_single21_02;...
    max_deviation_single21_03,avg_deviation_single21_03;...
    max_deviation_single21_04,avg_deviation_single21_04];
% deviations = bar(X,Y);
deviations = bar(X,Y);
digits(4)
xtips1 = double(deviations(1).XEndPoints);
ytips1 = deviations(1).YEndPoints;
labels1 = num2str(ytips1','%20.4f');
text(xtips1,ytips1,labels1,'HorizontalAlignment','center',...
    'VerticalAlignment','bottom','FontSize', 24)
xtips2 = deviations(2).XEndPoints;
ytips2 = deviations(2).YEndPoints;
labels2 = num2str(ytips2','%20.4f');
text(xtips2,ytips2,labels2,'HorizontalAlignment','center',...
    'VerticalAlignment','bottom','FontSize', 24)
xtips3 = deviations(3).XEndPoints;
ytips3 = deviations(3).YEndPoints;
labels3 = num2str(ytips3','%20.4f');
text(xtips3,ytips3,labels3,'HorizontalAlignment','center',...
    'VerticalAlignment','bottom','FontSize', 24)
xtips4 = deviations(4).XEndPoints;
ytips4 = deviations(4).YEndPoints;
labels4 = num2str(ytips4','%20.4f');
text(xtips4,ytips4,labels4,'HorizontalAlignment','center',...
    'VerticalAlignment','bottom','FontSize', 24)
% text(X,Y,num2str(Y'),'HorizontalAlignment','center','VerticalAlignment','bottom','FontSize', 20);
title('Max and Average Deviations for Different Trim Durations ','FontSize', 24)
ylim([0,0.25])
% xlabel('Types','FontSize', 24)
ylabel('Deviation[m]','FontSize', 24)
legend('0.1s','0.2s','0.3s','0.4s')
set(gca,'FontSize',24)


% priority assignment
% max number of vehicles
figure
X = categorical({'Constant','Random','FCA','Right-of-way'});
X = reordercats(X,{'Constant','Random','FCA','Right-of-way'});
Y = [4,8,6,20];
priority = bar(X,Y);
text(X,Y,num2str(Y'),'HorizontalAlignment','center','VerticalAlignment','bottom','FontSize', 24);
title('Max number of vehicles running for 180 seconds ','FontSize', 24)
ylim([0,25])
% xlabel('Types','FontSize', 24)
ylabel('Number of Vehicles','FontSize', 24)
set(gca,'FontSize',24)

% max running time for 20 vehicles
figure
X = categorical({'Constant','Random','FCA','Right-of-way'});
X = reordercats(X,{'Constant','Random','FCA','Right-of-way'});
Y = [1.6,2.0,1.6,180];
priority = bar(X,Y);
text(X,Y,num2str(Y'),'HorizontalAlignment','center','VerticalAlignment','bottom','FontSize', 24);
title('Max running time for 20 vehicles','FontSize', 24)
ylim([0,190])
% xlabel('Types','FontSize', 24)
ylabel('Time [s]','FontSize', 24)
set(gca,'FontSize',24)





function [deviation_single,max_deviation_single,avg_deviation_single, avg_vRef_single, avg_speed_single] = plot_deviation(result)
trim_duration = result.scenario.dt;
iteration_structs = result.iteration_structs;
realpoints = result.trajectory_predictions;
total_steps = length(result.iteration_structs);
[deviation_single,max_deviation_single,avg_deviation_single] = deviation(iteration_structs,realpoints,total_steps,trim_duration);
[avg_vRef_single, avg_speed_single] = speed(iteration_structs,total_steps);
subplot(1,2,1)
hold on
plot(1:length(deviation_single), deviation_single)
plot(1:length(deviation_single),ones(1,length(deviation_single))*max_deviation_single)
ylim([0,0.02])
xlabel('Steps','FontSize', 24)
ylabel('Deviation [m]','FontSize', 20)
title('Deviations over 300 steps','FontSize', 24)
set(gca,'FontSize',20)

% subplot(1,2,2)
% hold on
% X = categorical({'Max deviation','Average Deviation'});
% X = reordercats(X,{'Max deviation','Average Deviation'});
% Y = [max_deviation_single,avg_deviation_single];
% deviations = bar(X,Y);
% deviations = bar(X,Y,'stacked');
% text(X,Y,num2str(Y'),'HorizontalAlignment','center','VerticalAlignment','bottom','FontSize', 20);
% title('Max and Average Deviation ','FontSize', 24)
% ylim([0,0.02])
% xlabel('Types','FontSize', 24)
% ylabel('Deviation[m]','FontSize', 24)
% set(gca,'FontSize',20)
end

function [deviation,max_deviation,avg_deviation] = deviation(iteration_structs,realpoints,total_steps,trim_duration)
    nveh=size(iteration_structs,1);
    scenario = Scenario();
    time_per_tick = scenario.time_per_tick;
    tick_per_step = trim_duration/time_per_tick;
    deviation = zeros(nveh,total_steps);
    for n = 1:nveh

        for istep = 1:total_steps
            refpoints = iteration_structs{1,istep}.referenceTrajectoryPoints;
            current_refpoint_x = refpoints(n,1,1);
            current_refpoint_y = refpoints(n,1,2);
            current_refpoint = [current_refpoint_x, current_refpoint_y];

            current_realpoint_x = realpoints{n,istep}(tick_per_step+1,1);
            current_realpoint_y = realpoints{n,istep}(tick_per_step+1,2);
            current_realpoint = [current_realpoint_x, current_realpoint_y];
            deviation(n,istep)=norm((current_refpoint-current_realpoint),2);
        end

    end

    max_deviation = zeros(1,nveh);
    avg_deviation = zeros(1,nveh);
    for n = 1:nveh
        max_deviation(1,n) = max(deviation(n,:));
        avg_deviation(1,n) = mean(deviation(n,:));
    end

end

function [avg_vRef, avg_speed] = speed(iteration_structs,total_steps)
    nveh=size(iteration_structs{1,1}.vRef,1);
    [trim_inputs, ~] = choose_trims(12);
    vRef = zeros(nveh,total_steps);
    vTrim = zeros(nveh,total_steps);
    avg_vRef = zeros(1,nveh);
    avg_speed = zeros(1,nveh);
    for n = 1:nveh

        for istep = 1:total_steps
            vRef(n,istep) = iteration_structs{1,istep}.vRef(n,1);
            vTrim(n,istep) = trim_inputs(iteration_structs{1,istep}.trim_indices(n),2);
        end 
        avg_vRef(1,n) = mean(vRef(n,:));
        avg_speed(1,n) = mean(vTrim(n,:));
    end    
end












