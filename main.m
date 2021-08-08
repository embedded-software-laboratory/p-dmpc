function result = main(varargin)
% MAIN  main function for graph-based receeding horizon control

close all
clc

% Determine options
switch nargin
    case 3
        options = selection(varargin{1},varargin{2},varargin{3});
    case 2
        options = selection(varargin{1},varargin{2},1);
    case 1
        options = selection(varargin{1},2,1);
    otherwise
        options = selection();
end

% Setup scenario
scenario = lanelet_scenario3(0);

doPlotOnline = options.visu(1);
doPlotExploration = options.visu(2);
result = run_simulation(scenario, doPlotOnline, doPlotExploration);
end