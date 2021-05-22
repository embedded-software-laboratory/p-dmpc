function result = main(varargin)
close all
clc

% Determine options
if nargin == 3
    options = selection(varargin{1},varargin{2},varargin{3});
else
    if nargin==2
        options = selection(varargin{1},varargin{2},1);
    else
        if nargin==1
            options = selection(varargin{1},2,1);
        else
            options = selection();
        end
    end
end

% Setup scenario
scenario = Scenario(options);

doPlotOnline = options.visu(1);
doPlotExploration = options.visu(2);
result = run_simulation(scenario, doPlotOnline, doPlotExploration);
end