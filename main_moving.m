function result = main_moving()
close all
clc

% Setup scenario
scenario = moving_obstacle_scenario();

doPlotOnline = true;
doPlotExploration = false;
result = run_simulation(scenario, doPlotOnline, doPlotExploration);
end