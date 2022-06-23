function plotOnline(result,step_idx,tick_now,exploration,exp)
% PLOTONLINE    Plot function used for plotting the simulation state in a specified tick
%               during a specified time step

    scenario = result.scenario;
    iter = result.iteration_structs{step_idx};
    priority_list = result.priority(:,step_idx);

    % show description of hotkey
    find_text_hotkey = findall(gcf,'Type','text','Tag','hotkey');
    if isempty(find_text_hotkey)
        HotkeyDesc = {'Hotkey:';
                      'p: show/hide priority colorbar';
                      'i: show/hide vehicle IDs';
                      'c: show/hide coupling lines';
                      'w: show/hide coupling weights';
                      'space: pause/start simulation';
                      'esc: end simulation'};
        text(scenario.plot_limits(1,1)-1.5, scenario.plot_limits(2,2)-0.5, HotkeyDesc, 'FontSize',12, 'Tag','hotkey');
    end

    if isempty(exp.visu.colormap)
        exp.visu.colormap = colormap("hot");
    end

    % get colors
    n_priorities = length(unique(priority_list)); % number of different priorities
    n_colors_min = 6; % minimum number of colors
    n_colors = max(n_colors_min,n_priorities); 
    sticks = round(linspace(1,size(exp.visu.colormap,1),n_colors));
    vehColor = exp.visu.colormap(sticks,:); % evenly sample from colormap
    
    if exp.visu.isShowPriority
        if isempty(exp.visu.colorbar)
            exp.visu.colorbar = colorbar;
            exp.visu.colorbar.Title.String = '              Priority \newline(low value for high priority)'; % todo: find way to center the first line instead of using many spaces
            exp.visu.colorbar.Title.FontSize = 12;
        else
            exp.visu.colorbar.Visible = 'on';
        end
        clim([1 n_colors]) % define range of colorbar
        exp.visu.colorbar.Ticks = 1:n_colors; % only show integer ticks
    else
        if ~isempty(exp.visu.colorbar)
            exp.visu.colorbar.Visible = 'off';
        end
    end

    if nargin < 3
        tick_now = 1;
    end

    if isempty(exploration)
        exploration.doExploration = false;
    end



    nVeh = scenario.nVeh;
    nObst = size(scenario.obstacles,2);
    nDynObst = size(scenario.dynamic_obstacle_fullres,1);
    
    set(0,'DefaultTextFontname', 'Verdana');
    set(0,'DefaultAxesFontName', 'Verdana');
    
    
    %% Simulation state / scenario plot

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
            plot_lanelets(scenario.lanelets,scenario.name);
        end
    end
% plot_lanelets(scenario.lanelets);

    %%

    % Sampled trajectory points
    for v=1:nVeh
        line(   iter.referenceTrajectoryPoints(v,:,1), ...
                iter.referenceTrajectoryPoints(v,:,2), ...
                'Color',vehColor(priority_list(v),:),'LineStyle','none','Marker','o','MarkerFaceColor',vehColor(priority_list(v),:),'MarkerSize',3,'LineWidth',1 );
    end

    % predicted trajectory
    for v=1:nVeh
        line(   result.trajectory_predictions{v,step_idx}([1:scenario.tick_per_step+1:end,end],1), ...
                result.trajectory_predictions{v,step_idx}([1:scenario.tick_per_step+1:end,end],2), ...
                'Color',vehColor(priority_list(v),:),'LineStyle','none','Marker','+','MarkerFaceColor',vehColor(priority_list(v),:),'MarkerSize', 3, 'LineWidth',1 );
                % Matlab R2021a:
                %'Color',vehColor(priority_list(v),:),'LineStyle','none','Marker','|','MarkerFaceColor',vehColor(priority_list(v),:),'MarkerSize', 3, 'LineWidth',1 );
                % Matlab R2020a:
                %'Color',vehColor(priority_list(v),:),'LineStyle','none','Marker','+','MarkerFaceColor',vehColor(priority_list(v),:),'MarkerSize', 3, 'LineWidth',1 );
        line(   result.trajectory_predictions{v,step_idx}(:,1), ...
                result.trajectory_predictions{v,step_idx}(:,2), ...
                'Color',vehColor(priority_list(v),:),'LineWidth',1 );
    end

    % Vehicle rectangles
    for v=1:nVeh
        veh = scenario.vehicles(v);
        pos_step = result.trajectory_predictions{v,step_idx};
        x = pos_step(tick_now,:);
        vehiclePolygon = transformedRectangle(x(1),x(2),x(3), veh.Length,veh.Width);
        patch(   vehiclePolygon(1,:)...
                ,vehiclePolygon(2,:)...
                ,vehColor(priority_list(v),:)...
                ,'LineWidth', 1 ...
        );

        % plot the priority
        if exp.visu.isShowPriority
            text(x(1),x(2),num2str(result.priority(v,step_idx)),'FontSize', 12, 'LineWidth',1,'Color','m');
        end

        % plot the vehicle index
%         if exp.visu.isShowVehID
%             text(x(1)+0.1,x(2)+0.1,num2str(v),'FontSize', 16, 'LineWidth',1,'Color','b');
%         end

        % plot the vehicle ID
        if exp.visu.isShowVehID
            text(x(1)+0.1,x(2)+0.1,num2str(veh.ID),'FontSize', 12, 'LineWidth',1,'Color','b');
        end
%         
    end

    % plot scenario adjacency
    if exp.visu.isShowCoupling
        x0 = cellfun(@(c)c(tick_now,:), result.trajectory_predictions(:,step_idx), 'UniformOutput', false);
        x0 = cell2mat(x0);
        if ~isempty(scenario.coupling_weights)
            plot_coupling_lines(scenario.coupling_weights, x0, scenario.belonging_vector, scenario.coupling_info, 'ShowWeights', exp.visu.isShowWeight)
        else
            plot_coupling_lines(scenario.directed_coupling, x0, [], [], 'ShowWeights', exp.visu.isShowWeight)
        end
    end

%     plot distance
%     text((iter.x0(v,1)+iter.x0(adj_v,1))/2,(iter.x0(v,2)+iter.x0(adj_v,2))/2,...
%         num2str(round(result.distance(v,adj_v,step_idx),2)),'FontSize', 12, 'LineWidth',1,'Color','b');

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
    
    t=title(sprintf('Scenario: \\verb!%s!, Optimizer: \\verb!%s!, Strategy: \\verb!%s!, \nStep: %i, Time: %3.1fs',...
        scenarioName,...
        optimizer,...
        strategy,...
        step_idx,...
        (step_idx-1)*scenario.dt + (tick_now-1) * scenario.time_per_tick),'Interpreter','latex','FontSize',12);

    set(t,'HorizontalAlignment', 'center');
        
    drawnow
end


