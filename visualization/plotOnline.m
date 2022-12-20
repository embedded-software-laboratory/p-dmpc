function plotOnline(result,step_idx,tick_now,exploration,visu)
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
    priority_list = result.priority_list(:,step_idx);

    nVeh = scenario.options.amount;
    nObst = size(scenario.obstacles,2);
    nDynObst = size(iter.dynamic_obstacle_fullres,1);
    
    set(0,'DefaultTextFontname', 'Verdana');
    set(0,'DefaultAxesFontName', 'Verdana');

    if nargin < 3
        tick_now = 1;
    end

    if isempty(exploration)
        exploration.doExploration = false;
    end    
    
    %% Simulation state / scenario plot

    % find all the plots with property "LineWidth 1", which are different to plot_lanelets (default "LineWidth 0.5")  
    % at every time step, delete all these plots while keep plot_lanelets
    h = findobj('LineWidth', 1);
    delete(h);
    
    if exploration.doExploration
        visualize_exploration(exploration,scenario);
    end

    if visu.isVideoMode
        % in video mode, lanelets should be plotted at each time step
        hold on
        box on
        axis equal
        
        xlabel('\fontsize{14}{0}$x$ [m]','Interpreter','LaTex');
        ylabel('\fontsize{14}{0}$y$ [m]','Interpreter','LaTex');
    
        xlim(scenario.options.plot_limits(1,:));
        ylim(scenario.options.plot_limits(2,:));
        daspect([1 1 1])

        % plot the lanelets only once at the beginning
        if ~isempty(scenario.road_raw_data)
            plot_lanelets(scenario.road_raw_data.lanelet,scenario.name);
        end
    elseif step_idx == 1
        % if not video mode, lanelets should be plotted only at the initial time step
        hold on
        box on
        axis equal

        xlabel('\fontsize{14}{0}$x$ [m]','Interpreter','LaTex');
        ylabel('\fontsize{14}{0}$y$ [m]','Interpreter','LaTex');
    
        xlim(scenario.options.plot_limits(1,:));
        ylim(scenario.options.plot_limits(2,:));
        daspect([1 1 1])

        % plot the lanelets only once at the beginning
        if ~isempty(scenario.road_raw_data)
            if ~isempty(scenario.road_raw_data.lanelet)               
                plot_lanelets(scenario.road_raw_data.lanelet,scenario.options.scenario_name);
            end
        end
    end

    find_text_hotkey = findobj('Tag','hotkey');
    if visu.isShowHotkeyDescription
        % show description of hotkey
        if isempty(find_text_hotkey)
            HotkeyDesc = {'Hotkey:';
                          '{\itp}: show/hide priority colorbar';
                          '{\iti}: show/hide vehicle IDs';
                          '{\itc}: show/hide coupling lines';
                          '{\itw}: show/hide coupling weights';
                          '{\ith}: show/hide hot key descriptions';
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
            text(x_text_hotkey, y_text_hotkey, HotkeyDesc, 'FontSize',12, 'Tag','hotkey');
        end
    else
        % remove hot keys description if it was painted
        delete(find_text_hotkey);
    end
    % Define a new colormap in the first timestep, else get the colormap already associated with the plot.
    if step_idx == 1
        [priority_colormap, n_colors_max] = discrete_colormap();
        colormap(priority_colormap);
    else
        priority_colormap = get(gcf, 'Colormap');
        n_colors_max = size(priority_colormap, 1);
    end
 
    if visu.isShowPriority
        % Get plot's priority colorbar and set it to visible or define a new priority colorbar.
        priority_colorbar = findobj('Tag','priority_colorbar');
        if isempty(priority_colorbar)
            priority_colorbar = colorbar('Tag','priority_colorbar','FontName','Verdana','FontSize',9);
            priority_colorbar.Title.String = 'Priority';
            priority_colorbar.Title.FontSize = 9;
            priority_colorbar.TickLabels = string(1:n_colors_max);
            priority_colorbar.TickLength = 0;
        else
            priority_colorbar.Visible = 'on';
        end
        
        % Plot labels in the middle of each priority color.
        priority_colorbar.Ticks = 0.5:n_colors_max-0.5;
        % Define the range of the colorbar according to the number of colors.
        caxis([0 n_colors_max]);
%         clim([0 n_colors_max]) % renamed from caxis in R2022a
    end

    %%
    % Sampled trajectory points
    for v=1:nVeh
        line(   iter.referenceTrajectoryPoints(v,:,1), ...
                iter.referenceTrajectoryPoints(v,:,2), ...
                'Color',priority_colormap(priority_list(v),:),'LineStyle','none','Marker','o', ...
                'MarkerFaceColor',priority_colormap(priority_list(v),:),'MarkerSize',3,'LineWidth',1 );
    end

    % predicted trajectory
    for v=1:nVeh
        line(   result.trajectory_predictions{v,step_idx}([1:scenario.options.tick_per_step+1:end,end],1), ...
                result.trajectory_predictions{v,step_idx}([1:scenario.options.tick_per_step+1:end,end],2), ...
                'Color',priority_colormap(priority_list(v),:),'LineStyle','none','Marker','+', ...
                'MarkerFaceColor',priority_colormap(priority_list(v),:),'MarkerSize', 3, 'LineWidth',1 );
                % Matlab R2021a:
                %'Color',priority_colormap(priority_list(v),:),'LineStyle','none','Marker','|','MarkerFaceColor',priority_colormap(priority_list(v),:),'MarkerSize', 3, 'LineWidth',1 );
                % Matlab R2020a:
                %'Color',priority_colormap(priority_list(v),:),'LineStyle','none','Marker','+','MarkerFaceColor',priority_colormap(priority_list(v),:),'MarkerSize', 3, 'LineWidth',1 );
        line(   result.trajectory_predictions{v,step_idx}(:,1), ...
                result.trajectory_predictions{v,step_idx}(:,2), ...
                'Color',priority_colormap(priority_list(v),:),'LineWidth',1 );
    end

    % Vehicle rectangles
    for v=1:nVeh
        veh = scenario.vehicles(v);
        pos_step = result.trajectory_predictions{v,step_idx};
        x = pos_step(tick_now,:);
        vehiclePolygon = transformedRectangle(x(1),x(2),x(3), veh.Length,veh.Width);
        patch(   vehiclePolygon(1,:)...
                ,vehiclePolygon(2,:)...
                ,priority_colormap(priority_list(v),:)...
                ,'LineWidth', 1 ...
        );

        % plot the priority
%         if visu.isShowPriority
%             text(x(1),x(2),num2str(result.priority(v,step_idx)),'FontSize', 12, 'LineWidth',1,'Color','m');
%         end

        % plot the vehicle index in the middle of each vehicle on a lighter background
        if visu.isShowVehID
            radius = veh.Width * 0.95 / 2;
            rectangle('Position', [x(1)-radius, x(2)-radius, 2*radius, 2*radius], 'Curvature', [1,1], ...
                      'FaceColor', [1, 1, 1, 0.75], 'LineStyle', 'none', 'LineWidth', 1, 'Tag', 'circle');
            text(x(1), x(2), num2str(v),'FontSize', 10, 'LineWidth', 1, 'Color', 'black', 'HorizontalAlignment', 'center');
        end

        % plot the vehicle ID
%         if visu.isShowVehID
%             text(x(1)+0.1,x(2)+0.1,num2str(veh.ID),'FontSize', 12, 'LineWidth',1,'Color','b');
%         end
                
        if visu.isShowReachableSets 
            if scenario.options.bound_reachable_sets
                text_RS = 'Bounded reachable set by lanelet boundaries';
            else
                text_RS = 'Unbounded reachable set';
            end

            if isempty(visu.vehsReachableSets)
                [RS_x,RS_y] = boundary(result.iteration_structs{step_idx}.reachable_sets{v,scenario.options.Hp});
                line(RS_x,RS_y,'LineWidth',1.0,'Color','k');
                text(mean(RS_x),mean(RS_y),text_RS,'LineWidth',1,'FontSize',16)
            elseif ismember(v,visu.vehsReachableSets)
                % specify vehicles whose reachable sets should be shown
                [RS_x,RS_y] = boundary(result.iteration_structs{step_idx}.reachable_sets{v,scenario.options.Hp});
                line(RS_x,RS_y,'LineWidth',1.0,'Color','k');
                text(mean(RS_x),mean(RS_y),text_RS,'LineWidth',1,'FontSize',16)
            end
            
        end
        
        if visu.isShowLaneletCrossingAreas
            LCA = result.lanelet_crossing_areas{step_idx}{v};
            if ~isempty(LCA)
                if isempty(visu.vehsLaneletCorssingAreas)
                    LCAs_xy = [LCA{:}];
                    line(LCAs_xy(1,:),LCAs_xy(2,:),'LineWidth',1.0,'Color','k');
                elseif ismember(v,visu.vehsLaneletCorssingAreas)
                    % specify vehicles whose lanelet crossing areas should be shown
                    LCAs_xy = [LCA{:}];
                    line(LCAs_xy(1,:),LCAs_xy(2,:),'LineWidth',1.0,'Color','k');
                end
                text(max(LCAs_xy(1,:))+0.02,max(LCAs_xy(2,:)),'Lanelet crossing area','LineWidth',2,'FontSize',16)
            end
        end
    end

    % plot scenario adjacency
    if visu.isShowCoupling
        coupling_visu = struct('FontSize',9,'LineWidth',1,'isShowLine',visu.isShowCoupling,'isShowValue',visu.isShowWeight, 'radius', radius);
        x0 = cellfun(@(c)c(tick_now,:), result.trajectory_predictions(:,step_idx), 'UniformOutput', false);
        x0 = cell2mat(x0);
        if ~isempty(iter.coupling_weights_reduced)
            plot_coupling_lines(iter.coupling_weights_reduced, x0, result.belonging_vector(:,step_idx), result.coupling_info{step_idx}, coupling_visu)
        else
            plot_coupling_lines(result.directed_coupling{step_idx}, x0, [], [], coupling_visu)
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
        pos_step = iter.dynamic_obstacle_fullres{obs,step_idx};
        x = pos_step(tick_now,:);
        obstaclePolygon = transformedRectangle(x(1),x(2),pi/2, iter.dynamic_obstacle_shape(1),iter.dynamic_obstacle_shape(2));
        patch(   obstaclePolygon(1,:)...
                ,obstaclePolygon(2,:)...
                ,[0.5 0.5 0.5]...
                ,'LineWidth', 1 ...
        );
    end
    
    
    scenarioName = scenario.options.scenario_name;
    optimizer = 'Graph Search';
    strategy = scenario.controller_name;
    
    t=title(sprintf('Scenario: \\verb!%s!, Optimizer: \\verb!%s!, Strategy: \\verb!%s!, \nStep: %i, Time: %3.1fs',...
        scenarioName,...
        optimizer,...
        strategy,...
        step_idx,...
        (step_idx-1)*scenario.options.dt + (tick_now-1) * scenario.options.time_per_tick),'Interpreter','latex','FontSize',12);

    set(t,'HorizontalAlignment', 'center');
        
    drawnow
end


