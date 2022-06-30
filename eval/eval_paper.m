function eval_paper(plot_online,is_video_exported)
% EVAL_PAPER    Script yielding the evaluation results used in the paper

if nargin==0
    plot_online = 0;
    is_video_exported = 1;
end

%% *Trajectory planning*
% ------------------------------------------------------------------------------
clear results s
disp('Evaluating with one vehicle and moving obstacles.')
s(1) = moving_obstacle_scenario();
results(1) = run_simulation(s(1),plot_online,0);
% 11, 15, 19, 22

is_start_end = 1;
s(2) = moving_obstacle_scenario(is_start_end);
sub_controller = StartEndPlanner();
s(2).sub_controller = @sub_controller.run;
results(2) = run_simulation(s(2),plot_online,0);

nr = numel(results);

filepath_text = fullfile('results', 'rhgs_vs_gs.txt');
approaches = {'RHGS','SGS'};

%% Overview
fig = overviewPlot(results(1),[11, 15, 19, 22]);
overviewPlot(results(2),[11, 15, 19, 22],fig,1);

%% Computation time
runtimes = reshape([results.controller_runtime],[],nr);
t_max = max(runtimes);
t_median = median(runtimes);
t = [t_max; t_median];

% plot
fig = figure;
% X = categorical({'t_{max}','t_{median}'});
% X = reordercats(X,{'t_{max}','t_{median}'});
b = barh(1:2,t);
set(gca, 'XScale','log');
for i = 1:nr
    b(i).FaceColor = vehColor(i);
end
yticklabels({'t_{max}','t_{median}'})
legend(approaches{:},'Location','best')
xlabel('Computation Time [s]');

set(gca,'Ydir','reverse');
% ylim('tight')

set_figure_properties(fig, 'paper',3);
xmin = min(t,[],'all');
xmax = max(t,[],'all');
xlim([0.5*xmin, 1.5*xmax])
filepath = fullfile('results', 'rhgs_vs_gs_computation_time.pdf');
export_fig(fig, filepath);
close(fig);

fileID = fopen(filepath_text,'w');
for iApp = 1:numel(approaches)
    fprintf(fileID ...
        ,'%4s: t_max %7.2f ms (%5.1f times faster), t_median %7.2f ms (%3.1f times faster)\n' ...
        , approaches{iApp} ...
        , t_max(iApp)*1000 ...
        , t_max(2)/t_max(iApp) ...
        , t_median(iApp)*1000 ...
        , t_median(2)/t_median(iApp));
end
fclose(fileID);


%% Objective value
pos_ref = reshape( ...
    results(2).iteration_structs{1}.referenceTrajectoryPoints, ...
    size(results(2).iteration_structs{1}.referenceTrajectoryPoints,2), ...
    size(results(2).iteration_structs{1}.referenceTrajectoryPoints,3) ...
)';
% First reference point is meant for second actual point
pos_ref = pos_ref(:,1:end-1);

objective_value = zeros(1,nr);
for ir = 1:nr
    iter_struct_array = [results(ir).iteration_structs{:}];
    state = reshape([iter_struct_array.x0],numel(iter_struct_array(1).x0),[]);
    % First state point has no reference point
    pos_act = state(1:2,2:end);
    objective_value(ir) = sum(vecnorm(pos_ref-pos_act).^2);
end

% plot
fig = figure;
b = barh(1,objective_value);
for i = 1:nr
    b(i).FaceColor = vehColor(i);
end
legend('RHGS','SGS','Location','best')
xlabel('Objective Function Value');
yticks('');

ylim([0.5 2])

set(gca,'Ydir','reverse');

set_figure_properties(fig, 'paper',2.5);
filepath = fullfile('results', 'rhgs_vs_gs_objective_value.pdf');
export_fig(fig, filepath);
close(fig);

fileID = fopen(filepath_text,'a');
for iApp = 1:numel(approaches)
    fprintf(fileID ...
        ,'%4s: J %7.2f (%5.1f times worse)\n' ...
        , approaches{iApp} ...
        , objective_value(iApp) ...
        , objective_value(iApp)/objective_value(2) );
end
fclose(fileID);



%% Video
for r = results
    if is_video_exported
        exportVideo(r);
    end
end



%% Recursive feasibility
% ------------------------------------------------------------------------------
clear results s
disp([
    'Evaluating with only acceleration as control input, ',...
    'recursive feasibility ON.'
]);
is_ok = false;
is_recursively_feasible = true;
s = recursive_feasibility_scenario(is_recursively_feasible,is_ok);
results(1) = run_simulation(s,plot_online,0);

disp([
    'Evaluating with only acceleration as control input, ',...
    'recursive feasibility OFF.'
]);
is_recursively_feasible = false;
s = recursive_feasibility_scenario(is_recursively_feasible,is_ok);
results(2) = run_simulation(s,plot_online,0);

is_ok = true;
s = recursive_feasibility_scenario(is_recursively_feasible,is_ok);
results(3) = run_simulation(s,plot_online,0);

plot_scenario(results(1),'scenario_recursive_feasibility');
plot_distance_over_time(results);

%% Receding horizon
% ------------------------------------------------------------------------------
disp('Evaluating with combinations of obstacle distance and control horizon.');
eval_horizon_obstacle();

%% Multi-agent experiment
% ------------------------------------------------------------------------------
clear results s


approaches = {};
for nVeh = 3:-1:1
    fprintf('Evaluating with %d vehicles crossing a circle.\n',nVeh);
    s = circle_scenario(nVeh);
    results(nVeh) = run_simulation(s,plot_online,0);
    approaches{nVeh} = sprintf('%d-circle',nVeh);
    plot_scenario(results(nVeh));
    if is_video_exported
        exportVideo(results(nVeh));
    end
end


%% Overview
% 2 vehicles
% overviewPlot(results(1),[1,4,8,13]);
% 3 vehicles 
% overviewPlot(results(2),[1,5,8,13]);
overviewPlot(results(2),[1,5,8]);

%% Computation time
nr = numel(results);


runtimes = reshape([results.controller_runtime],[],nr);
t_max = max(runtimes);
t_median = median(runtimes);
t = [t_max; t_median];
n_max_vertices =  max(reshape([results.n_expanded],[],3));

% plot
fig = figure;
% X = categorical({'t_{max}','t_{median}'});
% X = reordercats(X,{'t_{max}','t_{median}'});
b = barh(1:2,t);
set(gca, 'XScale','log');
for i = 1:nr
    b(i).FaceColor = vehColor(i);
end
yticklabels({'t_{max}','t_{median}'})
legend(approaches{:},'Location','best')
xlabel('Computation Time [s]');

set(gca,'Ydir','reverse');
% ylim('tight')

set_figure_properties(fig, 'paper',4);
xmin = min(t,[],'all');
xmax = max(t,[],'all');
xlim([0.5*xmin, 1.5*xmax])
filepath = fullfile('results', 'circle_computation_time.pdf');
export_fig(fig, filepath);
close(fig);

filepath_text = fullfile('results', 'circle_computation_time.txt');

fileID = fopen(filepath_text,'w');
for iApp = 1:numel(approaches)
    fprintf(fileID ...
        ,'%9s: t_max %10.2f ms, t_median %7.2f ms, n_{expanded,max}: %d\n' ...
        , approaches{iApp} ...
        , t_max(iApp)*1000 ...
        , t_median(iApp)*1000 ...
        , n_max_vertices(iApp) ...
    );
end
fclose(fileID);
