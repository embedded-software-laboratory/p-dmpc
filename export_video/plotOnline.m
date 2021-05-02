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

    xlim([-22,22]);
    ylim([-7,7]);
    

    % Sampled trajectory points
    for v=1:nVeh
        plot( iter.referenceTrajectoryPoints(v,:,1),iter.referenceTrajectoryPoints(v,:,2),'o','MarkerFaceColor',vehColor(v),'MarkerEdgeColor',vehColor(v),'MarkerSize',3 )
    end

    % predicted trajectory
    for v=1:nVeh
        plot( result.trajectory_predictions{v,step_idx}(:,1),result.trajectory_predictions{v,step_idx}(:,2),'Color',vehColor(v) );
    end

    % Vehicle rectangles
    for v=1:nVeh
        pos_step = result.vehicle_path_fullres{v,step_idx};
        x = pos_step(tick_now,:);
        vehiclePolygon = transformedRectangle(x(1),x(2),x(3), scenario.model.Lr+scenario.model.Lf,scenario.model.W);
        fill(vehiclePolygon(1,:),vehiclePolygon(2,:),vehColor(v));
    end

%     % Vehicle safety circles
%     for v=1:nVeh
%         x = vehiclePositions(:,v);
%         phi = linspace(0,2*pi);
%         c=cos(phi);
%         s=sin(phi);
%         r=scenario.RVeh(v);
%         plot(x(idx.x)+r*c,x(idx.y)+r*s,'Color',colorVeh(v,:));
%     end
    
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
    optimizer = 'optimizer';
    strategy = 'rhc';
%     [scenarioName, optimizer, strategy] = rename_scenario_optimizer_strategy(result.scenario.Name, result.scenario.controllerName);
    
    t=title(sprintf('Scenario: \\verb!%s!, Optimizer: \\verb!%s!, Strategy: \\verb!%s!, \nStep: %i, Time: %3.1fs',...
        scenarioName,...
        optimizer,...
        strategy,...
        step_idx,...
        (step_idx-1)*0.4 + (tick_now-1) * 0.4/49),'Interpreter','LaTex');

    set(t,'HorizontalAlignment', 'center');
    
    drawnow
end

function [scenarioName, optimizer, strategy]=rename_scenario_optimizer_strategy(scenarioName, controllerName)

controllerParts = strsplit(controllerName,' ');
assert(length(controllerParts)==2);
optimizer=controllerParts{1};

if strcmp(controllerParts{2},'PB')
    strategy = 'PB-Non-Coop. DMPC';
elseif strcmp(controllerParts{2},'Centralized')
    strategy = 'Centralized MPC';
else
    strategy = controllerParts{2};
end

[n, count] = sscanf(scenarioName, 'Parallel %i');
if count
    scenarioName = sprintf('%i-Parallel',n);
    return
end

[n, count] = sscanf(scenarioName, 'Circle %i');
if count
    scenarioName = sprintf('%i-Circle',n);
    return
end

if strcmp(scenarioName, '2-way collision (NCD Oscillation)')
    scenarioName = '2-Circle';    
elseif strcmp(scenarioName, 'Crossing PB Example')
    scenarioName = 'Crossing';    
end

end


