function plotOnline(result,step_idx,tick_now)
    iter = result.iteration_structs{step_idx};
    if nargin < 3
        tick_now = 1;
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
    

    % Sampled trajectory points
    for v=1:nVeh
        plot( iter.referenceTrajectoryPoints(v,:,1),iter.referenceTrajectoryPoints(v,:,2),'o','MarkerFaceColor',vehColor(v),'MarkerEdgeColor',vehColor(v),'MarkerSize',3 )
    end

    % predicted trajectory
    for v=1:nVeh
        plot( result.trajectory_predictions{v,step_idx}([1:scenario.tick_per_step+1:end,end],1),result.trajectory_predictions{v,step_idx}([1:scenario.tick_per_step+1:end,end],2),'|','MarkerFaceColor',vehColor(v),'MarkerEdgeColor',vehColor(v),'MarkerSize',3 );
        plot( result.trajectory_predictions{v,step_idx}(:,1),result.trajectory_predictions{v,step_idx}(:,2),'Color',vehColor(v) );
    end

    % Vehicle rectangles
    for v=1:nVeh
        pos_step = result.vehicle_path_fullres{v,step_idx};
        x = pos_step(tick_now,:);
        vehiclePolygon = transformedRectangle(x(1),x(2),x(3), scenario.model.Lr+scenario.model.Lf,scenario.model.W);
        fill(vehiclePolygon(1,:),vehiclePolygon(2,:),vehColor(v));
    end
    
    % Obstacle rectangles
    for i = 1:nObst        
        obstaclePolygon = transformedRectangle(...
            obstaclePositions(i,idx.x),...
            obstaclePositions(i,idx.y),...
            scenario.obstacles(i,idx.heading),...
            scenario.obstacles(i,idx.length),...
            scenario.obstacles(i,idx.width));
        %fill(obstaclePolygon(1,:),obstaclePolygon(2,:),colorObst(min(i,size(colorObst,1)),:));
        fill(obstaclePolygon(1,:),obstaclePolygon(2,:),[0 0 0]);
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


