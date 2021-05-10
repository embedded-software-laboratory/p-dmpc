function plotOnline(result,step_idx,tick_now,exploration)
    iter = result.iteration_structs{step_idx};
    if nargin < 3
        tick_now = 1;
    end
    if isempty(exploration)
        exploration.doExploration = false;
    end

    scenario = result.scenario;

    nVeh = scenario.nVeh;
    nObst = size( scenario.obstacles,1);
    Hp = scenario.Hp;
    
    set(0,'DefaultTextFontname', 'Verdana');
    set(0,'DefaultAxesFontName', 'Verdana');
    
    
    %% Simulation state / scenario plot
    %subplot(nVeh,3,[2 3*nVeh]);
    cla
    hold on
    box on
    axis equal
    
    xlabel('\fontsize{14}{0}$x$ [m]','Interpreter','LaTex');
    ylabel('\fontsize{14}{0}$y$ [m]','Interpreter','LaTex');

    xlim(result.plot_limits(1,:));
    ylim(result.plot_limits(2,:));
    
    if exploration.doExploration
        visualize_exploration(exploration,scenario);
    end
    

    % Sampled trajectory points
    for v=1:nVeh
        line(   iter.referenceTrajectoryPoints(v,:,1), ...
                iter.referenceTrajectoryPoints(v,:,2), ...
                'Color',vehColor(v),'LineStyle','none','Marker','o','MarkerFaceColor',vehColor(v),'MarkerSize',3 );
    end

    % predicted trajectory
    for v=1:nVeh
        line(   result.trajectory_predictions{v,step_idx}([1:scenario.tick_per_step+1:end,end],1), ...
                result.trajectory_predictions{v,step_idx}([1:scenario.tick_per_step+1:end,end],2), ...
                'Color',vehColor(v),'LineStyle','none','Marker','|','MarkerFaceColor',vehColor(v),'MarkerSize', 2 );
        line(   result.trajectory_predictions{v,step_idx}(:,1), ...
                result.trajectory_predictions{v,step_idx}(:,2), ...
                'Color',vehColor(v),'LineWidth',1 );
    end

    % Vehicle rectangles
    for v=1:nVeh
        veh = scenario.vehicles(v);
        pos_step = result.vehicle_path_fullres{v,step_idx};
        x = pos_step(tick_now,:);
        vehiclePolygon = transformedRectangle(x(1),x(2),x(3), veh.Length,veh.Width);
        fill(vehiclePolygon(1,:),vehiclePolygon(2,:),vehColor(v));
    end
    
    % Obstacle rectangle
    for obs = 1:nObst
        patch( scenario.obstacles{obs}(1,:),scenario.obstacles{obs}(2,:), vehColor(nVeh+obs), 'LineWidth', 1 );
    end
    
    scenarioName = scenario.name;
    optimizer = 'Graph Search';
    strategy = scenario.controller_name;
    
    t=title(sprintf('Scenario: \\verb!%s!, Optimizer: \\verb!%s!, Strategy: \\verb!%s!, \nStep: %i, Time: %3.1fs',...
        scenarioName,...
        optimizer,...
        strategy,...
        step_idx,...
        (step_idx-1)*scenario.dt + (tick_now-1) * scenario.time_per_tick),'Interpreter','LaTex');

    set(t,'HorizontalAlignment', 'center');
    
    drawnow
end


