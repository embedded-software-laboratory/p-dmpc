function overviewPlot( result, step_indices )
close all
scenario = result.scenario;
assert(scenario.nVeh<=2)

scenario = result.scenario;

nVeh = scenario.nVeh;
nObst = size(scenario.obstacles,2);
nDynObst = size(scenario.dynamic_obstacle_fullres,1);

figure('visible','off','position',[100 100 600 630],'color',[1 1 1]);


% show predictions for multiple timesteps
for step = 1:4
    step_idx = step_indices(step);
    subplot(4,2,step);
    hold on
    box on
    % Obstacle rectangle
    for obs = 1:nObst
        patch(   scenario.obstacles{obs}(1,:)...
                ,scenario.obstacles{obs}(2,:)...
                ,[0.5 0.5 0.5]...
                ,'LineWidth', 1 ...
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
                ,'LineWidth', 1 ...
        );
    end
    % past trajectory
    for v=1:nVeh
        for iStep = 1:step_idx
            line(   result.vehicle_path_fullres{v,iStep}(:,1), ...
                    result.vehicle_path_fullres{v,iStep}(:,2), ...
                    'Color',vehColor(v),'LineWidth',1 );
        end
    end
    % predicted trajectory
    for v=1:nVeh
        line(   result.trajectory_predictions{v,step_idx}([1:scenario.tick_per_step+1:end,end],1), ...
                result.trajectory_predictions{v,step_idx}([1:scenario.tick_per_step+1:end,end],2), ...
                'Color',vehColor(v),'LineStyle','none','Marker','o','MarkerFaceColor',vehColor(v),'MarkerSize', 2 );
        line(   result.trajectory_predictions{v,step_idx}(:,1), ...
                result.trajectory_predictions{v,step_idx}(:,2), ...
                'Color',vehColor(v),'LineWidth',1 );
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
                ,'LineWidth', 1 ...
        );
    end
    daspect([1 1 1]);
    xlim(scenario.plot_limits(1,:));
    ylim(scenario.plot_limits(2,:));
    if step == 3 || step == 4
        xlabel('$x\ [m]$','Interpreter','LaTex')
    end
    if step == 1 || step == 3
        ylabel('$y\ [m]$','Interpreter','LaTex')
    end
    
    title(['Step ' num2str(step_idx)],'Interpreter','LaTex');
end


export_fig(['output/overviewPlot_' scenario.name '.pdf'],'-painters');    

end

