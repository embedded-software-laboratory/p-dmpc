function plotTrafficStatus(result,step_idx,tick_now,exploration,visu)
% PLOTONLINE    Plot function used for plotting the simulation state in a specified tick
%               during a specified time step
% 
% INPUT: 
%   result: simulation results
% 
%   step_idx: time step
% 
%   tick_now: tick index
% 
%   visu: struct with fields: 'isShowVehID', 'isShowPriority', 'isShowCoupling' and 'isShowWeight'
% 

    scenario = result.scenario;
    iter = result.iteration_structs{step_idx};
    priority_list = result.priority(:,step_idx);

    nVeh = scenario.options.amount;
    nObst = size(scenario.obstacles,2);
    nDynObst = size(scenario.dynamic_obstacle_fullres,1);
    
    if nargin < 3
        tick_now = 1;
    end 
    if isempty(exploration)
        exploration.doExploration = false;
    end    
    
    %% Simulation state / scenario plot

    % find all the plots with property "LineWidth 1", which are different to plot_lanelets (default "LineWidth 0.5")  
    % at every time step, delete all these plots while keep plot_lanelets
    h = findobj('LineWidth',0.2);
    delete(h)
    
    if exploration.doExploration
        visualize_exploration(exploration,scenario);
    end
    
    hold on
    box on
    axis equal
    
    xlabel('$x\:[m]$','Interpreter','LaTex');
    ylabel('$y\:[m]$','Interpreter','LaTex');

    xlim(scenario.options.plot_limits(1,:));
    ylim(scenario.options.plot_limits(2,:));
    daspect([1 1 1])

    % plot the lanelets only once at the beginning
    if ~isempty(scenario.lanelets)
        plot_lanelets(scenario.lanelets,scenario.name);
    end

    colormap("hot"); % set colormap


    if visu.isShowHotkeyDescription
        % show description of hotkey
        find_text_hotkey = findall(gcf,'Type','text','Tag','hotkey');
        if isempty(find_text_hotkey)
            HotkeyDesc = {'Hotkey:';
                          '{\itp}: show/hide priority colorbar';
                          '{\iti}: show/hide vehicle IDs';
                          '{\itc}: show/hide coupling lines';
                          '{\itw}: show/hide coupling weights';
                          '{\itspace}: pause/start simulation';
                          '{\itreturn}: disable/enable plotting';
                          '{\itesc}: end simulation'};
            if strcmp(scenario.name,'Commonroad')
                x_text_hotkey = scenario.options.plot_limits(1,1) - 1.5;
                y_text_hotkey = scenario.options.plot_limits(2,2) - 0.5;
            elseif strcmp(scenario.name,'Circle_scenario')
                x_text_hotkey = scenario.options.plot_limits(1,1) - 2.0;
                y_text_hotkey = scenario.options.plot_limits(2,2) - 0.5;
            else
                % to be define according to the specific scenario
                x_text_hotkey = scenario.options.plot_limits(1,1) - 1.5;
                y_text_hotkey = scenario.options.plot_limits(2,2) - 0.5;
            end
            text(x_text_hotkey, y_text_hotkey, HotkeyDesc, 'Tag','hotkey');
        end
    end

    get_colormap = get(gcf,'Colormap');

    % get colors
    n_priorities = length(unique(priority_list)); % number of different priorities
    n_colors_min = 5; % minimum number of colors
    n_colors = max(n_colors_min,n_priorities); 
    sticks = round(linspace(1,size(get_colormap,1),n_colors+1));
    vehColor = get_colormap(sticks(2:end),:); % evenly sample from colormap
    
    find_colorbar = findall(gcf,'Type','ColorBar','Tag','priority_colorbar');
    if visu.isShowPriority
        if isempty(find_colorbar)
            priority_colorbar = colorbar('Tag','priority_colorbar','FontName','Verdana','FontSize',9);
            priority_colorbar.Title.String = '              Priority \newline(low value for high priority)'; % todo: find way to center the first line instead of using many spaces
            priority_colorbar.Title.FontSize = 9;
            priority_colorbar.Ticks = 1:n_colors; % only show integer ticks
        else
            find_colorbar.Visible = 'on';
            find_colorbar.Ticks = 1:n_colors;
        end

        caxis([0 n_colors]); % define range of colorbar
%         clim([1 n_colors]) % renamed from caxis in R2022a
    else
        if ~isempty(find_colorbar)
            find_colorbar.Visible = 'off';
        end
    end

    %%
    % Sampled trajectory points
    for v=1:nVeh
        line(   iter.referenceTrajectoryPoints(v,:,1), ...
                iter.referenceTrajectoryPoints(v,:,2), ...
                'Color',vehColor(priority_list(v),:),'LineStyle','none','Marker','o','MarkerFaceColor',vehColor(priority_list(v),:),'MarkerSize',1,'LineWidth',0.2 );
    end

    % predicted trajectory
    for v=1:nVeh
        line(   result.trajectory_predictions{v,step_idx}([1:scenario.options.tick_per_step+1:end,end],1), ...
                result.trajectory_predictions{v,step_idx}([1:scenario.options.tick_per_step+1:end,end],2), ...
                'Color',vehColor(priority_list(v),:),'LineStyle','none','Marker','+','MarkerFaceColor',vehColor(priority_list(v),:),'MarkerSize', 1, 'LineWidth',0.2 );
                % Matlab R2021a:
                %'Color',vehColor(priority_list(v),:),'LineStyle','none','Marker','|','MarkerFaceColor',vehColor(priority_list(v),:),'MarkerSize', 3, 'LineWidth',0.2 );
                % Matlab R2020a:
                %'Color',vehColor(priority_list(v),:),'LineStyle','none','Marker','+','MarkerFaceColor',vehColor(priority_list(v),:),'MarkerSize', 3, 'LineWidth',0.2 );
        line(   result.trajectory_predictions{v,step_idx}(:,1), ...
                result.trajectory_predictions{v,step_idx}(:,2), ...
                'Color',vehColor(priority_list(v),:),'LineWidth',0.2 );
    end

    % Vehicle rectangles'FontSize', 9
    for v=1:nVeh
        veh = scenario.vehicles(v);
        pos_step = result.trajectory_predictions{v,step_idx};
        x = pos_step(tick_now,:);
        vehiclePolygon = transformedRectangle(x(1),x(2),x(3), veh.Length,veh.Width);
        patch(   vehiclePolygon(1,:)...
                ,vehiclePolygon(2,:)...
                ,vehColor(priority_list(v),:)...
                ,'LineWidth',0.2 ...
        );
    end

    % IDs, priorities
    for v=1:nVeh
        pos_step = result.trajectory_predictions{v,step_idx};
        x = pos_step(tick_now,:);

        % plot the priority
%         if visu.isShowPriority
%             text(x(1),x(2),num2str(result.priority(v,step_idx)), 'LineWidth',0.2,'Color','m');
%         end

        % plot the vehicle index
        if visu.isShowVehID
            text(x(1)+0.1,x(2)+0.1,num2str(v), 'LineWidth',0.2,'Color','b');
        end
        % plot the vehicle ID
%         if visu.isShowVehID
%             text(x(1)+0.1,x(2)+0.1,num2str(veh.ID), 'LineWidth',0.2,'Color','b');
%         end
    end

    % plot scenario adjacency
    if visu.isShowCoupling
        x0 = cellfun(@(c)c(tick_now,:), result.trajectory_predictions(:,step_idx), 'UniformOutput', false);
        x0 = cell2mat(x0);
        if ~isempty(scenario.coupling_weights_reduced)
            plot_coupling_lines(scenario.coupling_weights_reduced, x0, scenario.belonging_vector, scenario.coupling_info, 'ShowWeights', visu.isShowWeight)
        else
            plot_coupling_lines(scenario.directed_coupling, x0, [], [], 'ShowWeights', visu.isShowWeight)
        end
    end

%     plot distance
%     text((iter.x0(v,1)+iter.x0(adj_v,1))/2,(iter.x0(v,2)+iter.x0(adj_v,2))/2,...
%         num2str(round(result.distance(v,adj_v,step_idx),2)), 'LineWidth',0.2,'Color','b');

    % Obstacle rectangle
    for obs = 1:nObst
        patch(   scenario.obstacles{obs}(1,:)...
                ,scenario.obstacles{obs}(2,:)...
                ,[0.5 0.5 0.5]...
                ,'LineWidth',0.2 ...
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
                ,'LineWidth',0.2 ...
        );
    end
    
    
%     scenarioName = scenario.name;
%     optimizer = 'Graph Search';
%     strategy = scenario.controller_name;
%     
%     t=title(sprintf('Scenario: \\verb!%s!, Optimizer: \\verb!%s!, Strategy: \\verb!%s!, \nStep: %i, Time: %3.1fs',...
%         scenarioName,...
%         optimizer,...
%         strategy,...
%         step_idx,...
%         (step_idx-1)*scenario.options.dt + (tick_now-1) * scenario.options.time_per_tick),'Interpreter','latex','FontSize', 12);
% 
%     set(t,'HorizontalAlignment', 'center');
        
    drawnow
end


