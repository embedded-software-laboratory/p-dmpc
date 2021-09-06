function eval_paper(plot_online,is_video_exported)
% EVAL_PAPER    Script yielding the evaluation results used in the paper

if nargin==0
    plot_online = 0;
    is_video_exported = 1;
end

%% Trajectory planning
% ------------------------------------------------------------------------------
disp('Evaluating with one vehicle and moving obstacles.')
s = moving_obstacle_scenario();
r = run_simulation(s,plot_online,0);
overviewPlot(r,[17,21,25,28]);
if is_video_exported
    exportVideo(r);
end


%% Recursive feasibility
% ------------------------------------------------------------------------------
disp([
    'Evaluating with only acceleration as control input, ',...
    'recursive feasibility ON.'
]);
is_ok = false;
is_recursively_feasible = true;
s = recursive_feasibility_scenario(is_recursively_feasible,is_ok);
r(1) = run_simulation(s,plot_online,0);

disp([
    'Evaluating with only acceleration as control input, ',...
    'recursive feasibility OFF.'
]);
is_recursively_feasible = false;
s = recursive_feasibility_scenario(is_recursively_feasible,is_ok);
r(2) = run_simulation(s,plot_online,0);

is_ok = true;
s = recursive_feasibility_scenario(is_recursively_feasible,is_ok);
r(3) = run_simulation(s,plot_online,0);

plot_scenario(r(1));
plot_distance_over_time(r);

%% Receding horizon
% ------------------------------------------------------------------------------
disp('Evaluating with combinations of obstacle distance and control horizon.');
eval_horizon_obstacle();

%% Multi-agent experiment
% ------------------------------------------------------------------------------
r = [];
disp('Evaluating with two vehicles crossing a circle.');
s = circle_scenario(2);
r(1) = run_simulation(s,plot_online,0);
plot_scenario(r(1));
overviewPlot(r(1),[1,4,8,13]);
if is_video_exported
    exportVideo(r(1));
end

disp('Evaluating with three vehicles crossing a circle.');
s = circle_scenario(3);
r(2) = run_simulation(s,plot_online,0);
plot_scenario(r(2));
overviewPlot(r3,[1,5,8,13]);
if is_video_exported
    exportVideo(r(2));
end

computationTimePlotCentralized(r);
