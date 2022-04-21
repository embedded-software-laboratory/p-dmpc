function plotOnline(result,step_idx,tick_now,exploration)
% PLOTONLINE    Plot function used for plotting the simulation state in a specified tick
%               during a specified time step

    iter = result.iteration_structs{step_idx};
    if nargin < 3
        tick_now = 1;
    end
    if isempty(exploration)
        exploration.doExploration = false;
    end

    scenario = result.scenario;

    nVeh = scenario.nVeh;
    nObst = size(scenario.obstacles,2);
    nDynObst = size(scenario.dynamic_obstacle_fullres,1);
    
    set(0,'DefaultTextFontname', 'Verdana');
    set(0,'DefaultAxesFontName', 'Verdana');
    
    
    %% Simulation state / scenario plot
    %subplot(nVeh,3,[2 3*nVeh]);
%     cla % delete all the plots

    % find all the plots with property "LineWidth 1", which are different to plot_lanelets (default "LineWidth 0.5")  
    % at every time step, delete all these plots while keep plot_lanelets
    h = findobj('LineWidth',1);
    delete(h)
    
    hold on
    box on
    axis equal
    
    xlabel('\fontsize{14}{0}$x$ [m]','Interpreter','LaTex');
    ylabel('\fontsize{14}{0}$y$ [m]','Interpreter','LaTex');

    xlim(scenario.plot_limits(1,:));
    ylim(scenario.plot_limits(2,:));
    daspect([1 1 1])
    
    if exploration.doExploration
        visualize_exploration(exploration,scenario);
    end
    
    if step_idx == 1 % plot the lanelets only once at the beginning
        if ~isempty(scenario.lanelets)
            plot_lanelets(scenario.lanelets);
        end
    end
% plot_lanelets(scenario.lanelets);

    %%

    % Sampled trajectory points
    for v=1:nVeh
        line(   iter.referenceTrajectoryPoints(v,:,1), ...
                iter.referenceTrajectoryPoints(v,:,2), ...
                'Color',vehColor(v),'LineStyle','none','Marker','o','MarkerFaceColor',vehColor(v),'MarkerSize',3,'LineWidth',1 );
    end

    % predicted trajectory
    for v=1:nVeh
        line(   result.trajectory_predictions{v,step_idx}([1:scenario.tick_per_step+1:end,end],1), ...
                result.trajectory_predictions{v,step_idx}([1:scenario.tick_per_step+1:end,end],2), ...
                'Color',vehColor(v),'LineStyle','none','Marker','|','MarkerFaceColor',vehColor(v),'MarkerSize', 3, 'LineWidth',1 );
        line(   result.trajectory_predictions{v,step_idx}(:,1), ...
                result.trajectory_predictions{v,step_idx}(:,2), ...
                'Color',vehColor(v),'LineWidth',1 );
    end

    % Vehicle rectangles
    adj = cell(nVeh,nVeh);
    dis = cell(nVeh,nVeh);
    for v=1:nVeh
        veh = scenario.vehicles(v);
        pos_step = result.vehicle_path_fullres{v,step_idx};
        x = pos_step(tick_now,:);
        vehiclePolygon = transformedRectangle(x(1),x(2),x(3), veh.Length,veh.Width);
        patch(   vehiclePolygon(1,:)...
                ,vehiclePolygon(2,:)...
                ,vehColor(v)...
                ,'LineWidth', 1 ...
        );
        % plot the priority
        text(x(1),x(2),num2str(result.priority(v,step_idx)),'FontSize', 12, 'LineWidth',1);
        
        % plot the vehicle index
        text(x(1)+0.1,x(2)+0.1,num2str(v),'FontSize', 12, 'LineWidth',1,'Color','m');

        % plot scenario adjacency
        adjacent_vehicles = find(scenario.adjacency(v,:,end));
        adjacent_vehicles = adjacent_vehicles(adjacent_vehicles>v);
        for adj_v = adjacent_vehicles
            
            adj_pos_step = result.vehicle_path_fullres{adj_v,end};
            adj_x = adj_pos_step(tick_now,:);
            % plot adjacency
            adj{v,adj_v}=line([x(1),adj_x(1)],[x(2),adj_x(2)],'LineWidth',1, 'Color','r');
            
%             % plot distance
%             dis{v,adj_v}=text((iter.x0(v,1)+iter.x0(adj_v,1))/2,(iter.x0(v,2)+iter.x0(adj_v,2))/2,...
%                 num2str(result.distance(v,adj_v,step_idx)),'FontSize', 12, 'LineWidth',1,'Color','b');
 
        end
     
    end
    

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
        x = pos_step(tick_now,:);
        obstaclePolygon = transformedRectangle(x(1),x(2),pi/2, scenario.dynamic_obstacle_shape(1),scenario.dynamic_obstacle_shape(2));
        patch(   obstaclePolygon(1,:)...
                ,obstaclePolygon(2,:)...
                ,[0.5 0.5 0.5]...
                ,'LineWidth', 1 ...
        );
    end
        
    
    scenarioName = scenario.name;
    optimizer = 'Graph Search';
    strategy = scenario.controller_name;
    computation_levels = result.computation_levels(end);
    t=title(sprintf('Scenario: \\verb!%s!, Optimizer: \\verb!%s!, Strategy: \\verb!%s!, \nStep: %i, Time: %3.1fs, Computation Levels: %i',...
        scenarioName,...
        optimizer,...
        strategy,...
        step_idx,...
        (step_idx-1)*scenario.dt + (tick_now-1) * scenario.time_per_tick,...
        computation_levels),'Interpreter','LaTex');

    set(t,'HorizontalAlignment', 'center');
    
    
    drawnow
end


