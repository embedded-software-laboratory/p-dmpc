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

function plot_scenario(result)
fig = figure('Visible','on');
daspect([1 1 1]);

xlim(result.scenario.plot_limits(1,:));
ylim(result.scenario.plot_limits(2,:));

% reference trajectory
for iVeh = 1:numel(result.scenario.vehicles)
    veh = result.scenario.vehicles(iVeh);
    line(   veh.referenceTrajectory(:,1), ...
            veh.referenceTrajectory(:,2), ...
            'Color',vehColor(iVeh),'LineStyle', '--', 'LineWidth',1 ...
    );
end

% vehicle rectangle
for iVeh = 1:numel(result.scenario.vehicles)
    veh = result.scenario.vehicles(iVeh);
    veh.plot(vehColor(iVeh));
end

% Obstacles
for o = result.scenario.obstacles
    oCont = o{:};
    patch(oCont(1,:),oCont(2,:),[0.5 0.5 0.5]);
end

xlabel('$x$ [m]')
ylabel('$y$ [m]')

filetype = 'pdf';
filepath = fullfile(result.output_path, ['scenario.' filetype]);
set_figure_properties(fig,'paper')
exportgraphics(fig, filepath, 'ContentType','vector');
close(fig);
end