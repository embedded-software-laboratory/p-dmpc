% MIT License
% 
% Copyright (c) 2021 Lehrstuhl Informatik 11 - RWTH Aachen University
% 
% Permission is hereby granted, free of charge, to any person obtaining a copy
% of this software and associated documentation files (the "Software"), to deal
% in the Software without restriction, including without limitation the rights
% to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
% copies of the Software, and to permit persons to whom the Software is
% furnished to do so, subject to the following conditions:
% 
% The above copyright notice and this permission notice shall be included in all
% copies or substantial portions of the Software.
% 
% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
% IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
% FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
% AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
% LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
% OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
% SOFTWARE.
% 
% This file is part of receding-horizon-graph-search.
% 
% Author: i11 - Embedded Software, RWTH Aachen University

if verLessThan('matlab','9.10')
    warning('Code was designed for MATLAB 9.10, prepare for incompatibilities')
end
plot_online = 0;

%% Trajectory planning
% ------------------------------------------------------------------------------
disp('Evaluating with one vehicle and moving obstacles.')
s = moving_obstacle_scenario();
r = run_simulation(s,plot_online,0);
overviewPlot(r,[17,21,25,28]);
exportVideo(r);


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
disp('Evaluating with two vehicles crossing a circle.');
s = circle_scenario(2);
r = run_simulation(s,plot_online,0);
plot_scenario(r);
exportVideo(r);

disp('Evaluating with three vehicles crossing a circle.');
s = circle_scenario(3);
r = run_simulation(s,plot_online,0);
plot_scenario(r);
exportVideo(r);