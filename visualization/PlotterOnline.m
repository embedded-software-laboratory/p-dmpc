classdef PlotterOnline < handle
    properties
        abort
        paused     
    end
    properties (Access = private)
        resolution
        fig
        plot_options % struct to store logical variables indicating whether to show vehicle ID/priority/coupling/coupling weights
    end
    methods
        function obj = PlotterOnline(plot_options)
            % variables for key press callback
            obj.paused = false;
            obj.abort = false;
            obj.resolution = [1920 1080];
            obj.plot_options = plot_options;
            obj.fig = figure(...
                'Visible','On'...
                ,'Color',[1 1 1]...
                ,'units','pixel'...
                ,'OuterPosition',[100 100 obj.resolution(1) obj.resolution(2)]...
                );
            set(gcf,'WindowKeyPressFcn',@obj.keyPressCallback);
            hold on
        end

        function plotOnline(obj, result,step_idx,tick_now,exploration)
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
            h = findobj('LineWidth',1);
            delete(h)

            if exploration.doExploration
                visualize_exploration(exploration,scenario);
            end

            if obj.plot_options.isVideoMode
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
                if ~isempty(scenario.road_raw_data.lanelet)
                    plot_lanelets(scenario.road_raw_data.lanelet,scenario.name);
                end

                colormap("hot"); % set colormap
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
                if ~isempty(scenario.road_raw_data.lanelet)
                    plot_lanelets(scenario.road_raw_data.lanelet,scenario.options.scenario_name);
                end

                colormap("hot"); % set colormap
            end

            if obj.plot_options.isShowHotkeyDescription
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
                    text(x_text_hotkey, y_text_hotkey, HotkeyDesc, 'FontSize',12, 'Tag','hotkey');
                end
            end

            get_colormap = get(gcf,'Colormap');

            % get colors
            n_priorities = length(unique(priority_list)); % number of different priorities
            n_colors_min = 5; % minimum number of colors
            n_colors = max(n_colors_min,n_priorities);
            sticks = round(linspace(1,size(get_colormap,1),n_colors+1));
            vehColor = get_colormap(sticks(2:end),:); % evenly sample from colormap

            if obj.plot_options.isShowPriority
                find_colorbar = findall(gcf,'Type','ColorBar','Tag','priority_colorbar');
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
            end

            %%
            % Sampled trajectory points
            for v=1:nVeh
                line(   iter.referenceTrajectoryPoints(v,:,1), ...
                    iter.referenceTrajectoryPoints(v,:,2), ...
                    'Color',vehColor(priority_list(v),:),'LineStyle','none','Marker','o','MarkerFaceColor',vehColor(priority_list(v),:),'MarkerSize',3,'LineWidth',1 );
            end

            % predicted trajectory
            for v=1:nVeh
                line(   result.trajectory_predictions{v,step_idx}([1:scenario.options.tick_per_step+1:end,end],1), ...
                    result.trajectory_predictions{v,step_idx}([1:scenario.options.tick_per_step+1:end,end],2), ...
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
                %         if obj.plot_options.isShowPriority
                %             text(x(1),x(2),num2str(result.priority(v,step_idx)),'FontSize', 12, 'LineWidth',1,'Color','m');
                %         end

                % plot the vehicle index
                if obj.plot_options.isShowVehID
                    text(x(1)+0.1,x(2)+0.1,num2str(v),'FontSize', 16, 'LineWidth',1,'Color','b');
                end

                % plot the vehicle ID
                %         if obj.plot_options.isShowVehID
                %             text(x(1)+0.1,x(2)+0.1,num2str(veh.ID),'FontSize', 12, 'LineWidth',1,'Color','b');
                %         end

                if obj.plot_options.isShowReachableSets
                    if scenario.options.bound_reachable_sets
                        text_RS = 'Bounded reachable set by lanelet boundaries';
                    else
                        text_RS = 'Unbounded reachable set';
                    end

                    if isempty(obj.plot_options.vehsReachableSets)
                        [RS_x,RS_y] = boundary(result.iteration_structs{step_idx}.reachable_sets{v,scenario.options.Hp});
                        line(RS_x,RS_y,'LineWidth',1.0,'Color','k');
                        text(mean(RS_x),mean(RS_y),text_RS,'LineWidth',1,'FontSize',16)
                    elseif ismember(v,obj.plot_options.vehsReachableSets)
                        % specify vehicles whose reachable sets should be shown
                        [RS_x,RS_y] = boundary(result.iteration_structs{step_idx}.reachable_sets{v,scenario.options.Hp});
                        line(RS_x,RS_y,'LineWidth',1.0,'Color','k');
                        text(mean(RS_x),mean(RS_y),text_RS,'LineWidth',1,'FontSize',16)
                    end

                end

                if obj.plot_options.isShowLaneletCrossingAreas
                    LCA = result.lanelet_crossing_areas{step_idx}{v};
                    if ~isempty(LCA)
                        if isempty(obj.plot_options.vehsLaneletCorssingAreas)
                            LCAs_xy = [LCA{:}];
                            line(LCAs_xy(1,:),LCAs_xy(2,:),'LineWidth',1.0,'Color','k');
                        elseif ismember(v,obj.plot_options.vehsLaneletCorssingAreas)
                            % specify vehicles whose lanelet crossing areas should be shown
                            LCAs_xy = [LCA{:}];
                            line(LCAs_xy(1,:),LCAs_xy(2,:),'LineWidth',1.0,'Color','k');
                        end
                        text(max(LCAs_xy(1,:))+0.02,max(LCAs_xy(2,:)),'Lanelet crossing area','LineWidth',2,'FontSize',16)
                    end
                end
            end

            % plot scenario adjacency
            coupling_visu = struct('FontSize',9,'LineWidth',1,'isShowLine',obj.plot_options.isShowCoupling,'isShowValue',obj.plot_options.isShowWeight);
            if obj.plot_options.isShowCoupling
                x0 = cellfun(@(c)c(tick_now,:), result.trajectory_predictions(:,step_idx), 'UniformOutput', false);
                x0 = cell2mat(x0);
                if ~isempty(scenario.coupling_weights_reduced)
                    plot_coupling_lines(result.coupling_weights_reduced{step_idx}, x0, result.belonging_vector(:,step_idx), result.coupling_info{step_idx}, coupling_visu)
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
                pos_step = scenario.dynamic_obstacle_fullres{obs,step_idx};
                x = pos_step(tick_now,:);
                obstaclePolygon = transformedRectangle(x(1),x(2),pi/2, scenario.dynamic_obstacle_shape(1),scenario.dynamic_obstacle_shape(2));
                patch(   obstaclePolygon(1,:)...
                    ,obstaclePolygon(2,:)...
                    ,[0.5 0.5 0.5]...
                    ,'LineWidth', 1 ...
                    );
            end


            scenarioName = scenario.options.scenario_name;
            optimizer = 'Graph Search';
            % TODO
            strategy = 'controllerNamehere';

            t=title(sprintf('Scenario: \\verb!%s!, Optimizer: \\verb!%s!, Strategy: \\verb!%s!, \nStep: %i, Time: %3.1fs',...
                scenarioName,...
                optimizer,...
                strategy,...
                step_idx,...
                (step_idx-1)*scenario.options.dt + (tick_now-1) * scenario.options.time_per_tick),'Interpreter','latex','FontSize',12);

            set(t,'HorizontalAlignment', 'center');

            drawnow
        end
        function close_figure(obj)
            close(obj.fig);
        end

    end
    
    methods (Access = private)
        function keyPressCallback(obj, ~, eventdata)
            switch eventdata.Key
                case 'escape'
                    obj.abort = true;
                case 'space'
                    obj.paused = ~obj.paused;
                    if obj.paused 
                        disp('Pause simulation.')
                    else
                        disp('Start simulation.')
                    end
                case 'i'
                    obj.plot_options.isShowVehID = ~obj.plot_options.isShowVehID;
                    if obj.plot_options.isShowVehID 
                        disp('Show vehicle.')
                    else
                        disp('Hide Vehicle IDs.')
                    end
                case 'p'
                    obj.plot_options.isShowPriority = ~obj.plot_options.isShowPriority;
                    if obj.plot_options.isShowPriority 
                        disp('Show vehicle priorities.')
                    else
                        disp('Hide vehicle priorities.')
                        find_colorbar = findall(gcf,'Type','ColorBar','Tag','priority_colorbar');
                        if ~isempty(find_colorbar)
                            find_colorbar.Visible = 'off';
                        end
                    end
                case 'c'
                    obj.plot_options.isShowCoupling = ~obj.plot_options.isShowCoupling;
                    if obj.plot_options.isShowCoupling 
                        disp('Show couplings lines.')
                    else
                        disp('Hide couplings lines.')
                    end
                case 'w'
                    obj.plot_options.isShowWeight = ~obj.plot_options.isShowWeight;
                    if obj.plot_options.isShowWeight 
                        disp('Show couplings weights.')
                    else
                        disp('Hide couplings weights.')
                    end
                case 'return'
                    obj.doOnlinePlot = ~obj.doOnlinePlot;
                    if obj.doOnlinePlot 
                        disp('Enable plotting.')
                    else
                        disp('Disable Plotting.')
                    end
            end
        end
    end
end
