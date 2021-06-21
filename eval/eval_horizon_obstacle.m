function result = eval_horizon_obstacle(result)
% EVAL_HORIZON_OBSTACLE     Evaluate results yielded by the horizon obstacle scenario.

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