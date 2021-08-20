function result = main(varargin)
% MAIN  main function for graph-based receeding horizon control

if ~verLessThan('matlab','9.10')
    warning("Code is developed in MATLAB 2021a, prepare for backward incompatibilities.")
else

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
scenario = circle_scenario(options.amount,options.isPB);

doPlotOnline = options.visu(1);
doPlotExploration = options.visu(2);
result = run_simulation(scenario, doPlotOnline, doPlotExploration);
end
