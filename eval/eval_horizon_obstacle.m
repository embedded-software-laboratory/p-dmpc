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

function result = eval_horizon_obstacle(result)
horizons = 5:10:30;
obstacle_distances = 1:1:3;
if nargin==0
    result = cell(numel(horizons), numel(obstacle_distances));
    for i_N = 1:numel(horizons)
        N = horizons(i_N);
        for i_d_obstacle = 1:numel(obstacle_distances)
            d_obstacle = obstacle_distances(i_d_obstacle);
            fprintf('Simulating with N=%i, d=%f\n', N, d_obstacle);
            scenario = horizon_obstacle_scenario(N, d_obstacle);
            result{i_N, i_d_obstacle} = run_simulation(scenario, false, false);
        end
    end
end

[X,Y] = meshgrid(obstacle_distances, horizons);
Z = zeros(size(X));
for i_N = 1:numel(horizons)
    for i_d_obstacle = 1:numel(obstacle_distances)
        Z(i_N,i_d_obstacle) = sum(result{i_N, i_d_obstacle}.n_expanded);
    end
end
fig = figure('Visible','Off');
surf(X,Y,Z, 'FaceColor', 'interp');
xlabel('Distance [$m$]'...
    ,'Rotation',15 ...
    ,'Position',[2, -8] ...
    ,'HorizontalAlignment','center' ...
)
ylabel('Horizon'...
    ,'Rotation',-25 ...
    ,'Position',[0.5, 15] ...
    ,'HorizontalAlignment','center' ...
)
zlabel('Expanded vertices')
filetype = 'pdf';
filepath = fullfile('results', ['eval_horizon_obstacle.' filetype]);
set_figure_properties(fig,'paper',6)
exportgraphics(fig, filepath, 'ContentType','vector');
close(fig);
end