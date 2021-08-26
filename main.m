function result = main(varargin)
% MAIN  main function for graph-based receeding horizon control

if verLessThan('matlab','9.10')
    warning("Code is developed in MATLAB 2021a, prepare for backward incompatibilities.")
end

% Determine options
if nargin==2
    options = selection(varargin{1},varargin{2});
else
    if nargin==1
        options = selection(varargin{1},2);
    else
        options = selection();
    end
end

% Setup scenario
scenario = circle_scenario(options.amount);

doPlotOnline = options.visu(1);
doPlotExploration = options.visu(2);
result = run_simulation(scenario, doPlotOnline, doPlotExploration);
end
