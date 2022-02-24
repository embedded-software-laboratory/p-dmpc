load('/home/bingyi/Desktop/Master_thesis/Trajectory_planning/graph_based_planning/results/Commonroad_RHC-PB/data.mat')
% %% computation levels
% computation_levels = last4.computation_levels;
% unique_computation_levels = unique(computation_levels);
% n_unique_computation_levels = zeros(1,length(unique_computation_levels));
% 
% for i = 1:length(unique_computation_levels)
%     n_unique_computation_levels(i) = sum(computation_levels == unique_computation_levels(i));
%    
% end
% 
% levels=bar(unique_computation_levels,n_unique_computation_levels);
% text(unique_computation_levels,n_unique_computation_levels',num2str(n_unique_computation_levels'),'HorizontalAlignment','center','VerticalAlignment','bottom');
% title('Distribution of Computation Levels 0.4s','FontSize', 14)
% xlabel('Computation Level','FontSize', 14)
% ylabel('Number of Computation Levels','FontSize', 14)

% %% expanded nodes
% expanded_nodes = result_04.n_expanded;
% max_expanded = max(expanded_nodes);
% min_expanded = min(expanded_nodes);
% avg_expanded = round(mean(expanded_nodes));
% 
% X = categorical({'max\_expanded','avg\_expanded'});
% X = reordercats(X,{'max\_expanded','avg\_expanded'});
% Y = [max_expanded,avg_expanded];
% nodes = bar(X,Y);
% text(X,Y,num2str(Y'),'HorizontalAlignment','center','VerticalAlignment','bottom');
% title('Expanded Nodes (0.4s)','FontSize', 14)
% % xlabel('FontSize', 14)
% ylabel('Number of Expanded Nodes','FontSize', 14)


% %% controller time
% controller_runtime = result_04.controller_runtime;
% max_runtime = max(controller_runtime);
% min_runtime = min(controller_runtime);
% avg_runtime = mean(controller_runtime);
% 
% X = categorical({'max\_runtime','avg\_runtime'});
% X = reordercats(X,{'max\_runtime','avg\_runtime'});
% Y = [max_runtime,avg_runtime];
% runtime = bar(X,Y);
% text(X,Y,num2str(Y'),'HorizontalAlignment','center','VerticalAlignment','bottom');
% title('Controller Runtime (0.4s)','FontSize', 14)
% % xlabel('FontSize', 14)
% ylabel('Time','FontSize', 14)

%% deviation 0.2s for single vehicle
trim_duration = 0.2;
load('/home/bingyi/Desktop/Master_thesis/Trajectory_planning/graph_based_planning/Evaluation/result_single_vehicle.mat');
iteration_structs = result_single_vehicle.iteration_structs;
realpoints = result_single_vehicle.trajectory_predictions;
total_steps = length(result_single_vehicle.iteration_structs);
[deviation_single2,max_deviation_single2,avg_deviation_single2] = deviation(iteration_structs,realpoints,total_steps,trim_duration);
[avg_vRef_single2, avg_speed_single2] = speed(iteration_structs,total_steps);
bar(1:length(deviation_single2), deviation_single2)
ylim([0,0.02])
xlabel('Steps','FontSize', 14)
ylabel('deviation [m]','FontSize', 14)
title('Deviation for single vehicle with 0.2s trim duration','FontSize', 14)

% %% 
% % deviation 0.2s
% 
% trim_duration = 0.2;
% load('/home/bingyi/Desktop/Master_thesis/Trajectory_planning/graph_based_planning/Evaluation/result2.mat');
% iteration_structs = result2.iteration_structs;
% realpoints = result2.trajectory_predictions;
% total_steps = length(result2.iteration_structs);
% [deviation2,max_deviation2,avg_deviation2] = deviation(iteration_structs,realpoints,total_steps,trim_duration);
% [avg_vRef2, avg_speed2] = speed(iteration_structs,total_steps);
% 
% % deviation 0.3s
% trim_duration = 0.3;
% load('/home/bingyi/Desktop/Master_thesis/Trajectory_planning/graph_based_planning/Evaluation/result3.mat');
% iteration_structs = result3.iteration_structs;
% realpoints = result3.trajectory_predictions;
% total_steps = length(result3.iteration_structs);
% [deviation3,max_deviation3,avg_deviation3] = deviation(iteration_structs,realpoints,total_steps,trim_duration);
% [avg_vRef3, avg_speed3] = speed(iteration_structs,total_steps);
% 
% % deviation 0.4s
% trim_duration = 0.4;
% load('/home/bingyi/Desktop/Master_thesis/Trajectory_planning/graph_based_planning/Evaluation/result4.mat');
% iteration_structs = result4.iteration_structs;
% realpoints = result4.trajectory_predictions;
% total_steps = length(result4.iteration_structs);
% [deviation4,max_deviation4,avg_deviation4] = deviation(iteration_structs,realpoints,total_steps,trim_duration);
% [avg_vRef4, avg_speed4] = speed(iteration_structs,total_steps);
% 
% % deviation 0.5s
% trim_duration = 0.5;
% load('/home/bingyi/Desktop/Master_thesis/Trajectory_planning/graph_based_planning/Evaluation/result5.mat');
% iteration_structs = result5.iteration_structs;
% realpoints = result5.trajectory_predictions;
% total_steps = length(result5.iteration_structs);
% [deviation5,max_deviation5,avg_deviation5] = deviation(iteration_structs,realpoints,total_steps,trim_duration);
% [avg_vRef5, avg_speed5] = speed(iteration_structs,total_steps);
% 
% % plot deviation
% % X = categorical({'0.2s','0.3s','0.4s','0.5s'});
% % X = reordercats(X,{'0.2s','0.3s','0.4s','0.5s'});
% % Y = [avg_deviation2,avg_deviation3,avg_deviation4,avg_deviation5];
% 
% X = 1:20;
% % Y = avg_deviation2;
% Y = [avg_deviation2;avg_deviation3;avg_deviation4;avg_deviation5];
% deviations = bar(X,Y);
% % deviations = bar(X,Y,'stacked');
% % text(X,Y,num2str(Y'),'HorizontalAlignment','center','VerticalAlignment','bottom');
% title('Average Deviation ','FontSize', 14)
% xlabel('Vehicle','FontSize', 14)
% ylabel('Distance[m]','FontSize', 14)
% legend('0.2s','0.3s','0.4s','0.5s')
% 
% % plot average speed
% X = 1:20;
% Y = avg_speed2;
% % Y = [avg_speed2;avg_speed3;avg_speed4;avg_speed5];
% speeds = bar(X,Y);
% % deviations = bar(X,Y,'stacked');
% text(X,Y,num2str(Y'),'HorizontalAlignment','center','VerticalAlignment','bottom');
% title('Average Speed(0.2)','FontSize', 14)
% xlabel('Vehicle','FontSize', 14)
% ylabel('Speed[m/s]','FontSize', 14)
% % legend('0.2s','0.3s','0.4s','0.5s')
% 
% 
% 
% 
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
    nveh=size(iteration_structs,1);
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












