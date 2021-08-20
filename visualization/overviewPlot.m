function overviewPlot( result, step_indices )
% OVERVIEWPLOT  Export plot with multiple snapshots.

scenario = result.scenario;

nVeh = scenario.nVeh;
nObst = size(scenario.obstacles,2);
nDynObst = size(scenario.dynamic_obstacle_fullres,1);

fig = figure('visible','off','position',[100 100 600 630],'color',[1 1 1]);


% show predictions for multiple timesteps
for step = 1:numel(step_indices)
    step_idx = step_indices(step);
    subplot(numel(step_indices),1,step);
    hold on
    box on
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
    % past trajectory
    for v=1:nVeh
        for iStep = 1:step_idx
            line(   result.vehicle_path_fullres{v,iStep}(:,1), ...
                    result.vehicle_path_fullres{v,iStep}(:,2), ...
                    'Color',vehColor(v) ...
            );
        end
    end
    % predicted trajectory
    for v=1:nVeh
        line(   result.trajectory_predictions{v,step_idx}([1:scenario.tick_per_step+1:end,end],1), ...
                result.trajectory_predictions{v,step_idx}([1:scenario.tick_per_step+1:end,end],2), ...
                'Color',vehColor(v),'LineStyle','none','Marker','o','MarkerFaceColor',vehColor(v),'MarkerSize', 1 );
        line(   result.trajectory_predictions{v,step_idx}(:,1), ...
                result.trajectory_predictions{v,step_idx}(:,2), ...
                'Color',vehColor(v) );
    end
    % Vehicle rectangles
    for v=1:nVeh
        veh = scenario.vehicles(v);
        pos_step = result.vehicle_path_fullres{v,step_idx};
        x = pos_step(1,:);
        vehiclePolygon = transformedRectangle(x(1),x(2),x(3), veh.Length,veh.Width);
        patch(   vehiclePolygon(1,:)...
                ,vehiclePolygon(2,:)...
                ,vehColor(v)...
        );
    end
    daspect([1 1 1]);
    xlim(scenario.plot_limits(1,:));
    ylim(scenario.plot_limits(2,:));
    if step == numel(step_indices)
        xlabel('$x$ [m]','Interpreter','LaTex')
    end
    ylabel('$y$ [m]','Interpreter','LaTex')
        
    title(['Step ' num2str(step_idx)],'Interpreter','LaTex');
end

set_figure_properties(fig, 'paper', 12);
filepath = fullfile(result.output_path, 'overviewPlot.pdf');
export_fig(fig, filepath)
close(fig)
end
