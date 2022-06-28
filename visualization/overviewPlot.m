function fig = overviewPlot( result, step_indices, fig, colorOffset)
% OVERVIEWPLOT  Export plot with multiple snapshots.
arguments
    result (1,1)
    step_indices (1,:) double
    fig (1,1) matlab.ui.Figure = figure;%...
%         figure('visible','off','position',[100 100 600 630],'color',[1 1 1])
    colorOffset (1,1) double = 0
end


scenario = result.scenario;

nVeh = scenario.nVeh;
nObst = size(scenario.obstacles,2);
nDynObst = size(scenario.dynamic_obstacle_fullres,1);


nFigs = numel(step_indices);
if nargin<3
    tiledlayout(fig,nFigs,1);
end

% Oversize papersize to find fitting height
set_figure_properties(fig, 'paper', nFigs*5);

% show predictions for multiple timesteps
for step = 1:nFigs
    step_idx = step_indices(step);
    
    nexttile(step)
    hold on
    box on

    if nargin<3

        if step == numel(step_indices)
            xlabel('$x$ [m]','Interpreter','LaTex')
        end
        ylabel('$y$ [m]','Interpreter','LaTex')
            
        title(['Step ' num2str(step_idx)] ...
            ,'Interpreter','LaTex' ...
            ,'Units', 'normalized' ...
            ,'Position', [1.07, 0.5, 0] ...
            ,'VerticalAlignment','middle' ...
            ,'Rotation',70 ...
        );
    
        % Obstacle rectangle
        for obs = 1:nObst
            patch(   scenario.obstacles{obs}(1,:)...
                    ,scenario.obstacles{obs}(2,:)...
                    ,[0.5 0.5 0.5]...
            );
        end
        % dynamic obstacles
        for obs = 1:nDynObst
            pos_step = scenario.dynamic_obstacle_fullres{obs,step_idx};
            x = pos_step(1,:);
            obstaclePolygon = transformedRectangle(x(1),x(2),pi/2, scenario.dynamic_obstacle_shape(1),scenario.dynamic_obstacle_shape(2));
            patch(   obstaclePolygon(1,:)...
                    ,obstaclePolygon(2,:)...
                    ,[0.5 0.5 0.5]...
            );
        end
    end
    % past trajectory
    for v=1:nVeh
        for iStep = 1:step_idx
            line(   result.vehicle_path_fullres{v,iStep}(:,1), ...
                    result.vehicle_path_fullres{v,iStep}(:,2), ...
                    'Color',vehColor(v+colorOffset) ...
            );
        end
    end
    % predicted trajectory
    for v=1:nVeh
        line(   result.trajectory_predictions{v,step_idx}([1:scenario.tick_per_step+1:end,end],1), ...
                result.trajectory_predictions{v,step_idx}([1:scenario.tick_per_step+1:end,end],2), ...
                'Color',vehColor(v+colorOffset),'LineStyle','none','Marker','o','MarkerFaceColor',vehColor(v+colorOffset),'MarkerSize', 1 );
        line(   result.trajectory_predictions{v,step_idx}(:,1), ...
                result.trajectory_predictions{v,step_idx}(:,2), ...
                'Color',vehColor(v+colorOffset) );
    end
    % Vehicle rectangles
    for v=1:nVeh
        veh = scenario.vehicles(v);
        pos_step = result.vehicle_path_fullres{v,step_idx};
        x = pos_step(1,:);
        vehiclePolygon = transformedRectangle(x(1),x(2),x(3), veh.Length,veh.Width);
        patch(   vehiclePolygon(1,:)...
                ,vehiclePolygon(2,:)...
                ,vehColor(v+colorOffset)...
        );
    end
    daspect([1 1 1]);
    xlim(scenario.plot_limits(1,:));
    ylim(scenario.plot_limits(2,:));
end

set(fig.Children.Children(1),'Units','centimeters');
tile_height = fig.Children.Children(end).OuterPosition(4);
x_axis_label_height = fig.Children.Children(1).OuterPosition(4) - tile_height;
set(fig.Children.Children(1),'Units','normalized');
fig.Children.TileSpacing = 'compact';
set_figure_properties(fig, 'paper', nFigs*(tile_height+x_axis_label_height));
filepath = fullfile(result.output_path, 'overviewPlot.pdf');
export_fig(fig, filepath)
if nargout==0
    close(fig)
end
end
