classdef PlotterOnline < handle
    properties
        abort
        paused
    end
    properties (Access = private)
        resolution
        fig
        plot_options % struct to store logical variables indicating whether to show vehicle ID/priority/coupling/coupling weights
        scenario
        strategy
        vehicles
        nVeh
        timer % used to simulate real time plotting while receiving data from visualiuation queue
        simulation_time_before_pause
    end
    methods
        function obj = PlotterOnline(scenario)
            % variables for key press callback
            obj.paused = false;
            obj.abort = false;
            obj.resolution = [1920 1080];
            obj.plot_options = scenario.options.optionsPlotOnline;
            obj.scenario = scenario;
            obj.strategy = HLCFactory.get_controller_name(obj.scenario.options);
            obj.vehicles = scenario.vehicles;
            obj.nVeh = scenario.options.amount;
            obj.simulation_time_before_pause = 0 ;
            obj.fig = figure(...
                'Visible','On'...
                ,'Color',[1 1 1]...
                ,'units','pixel'...
                ,'OuterPosition',[100 100 obj.resolution(1) obj.resolution(2)]...
                );
            set(gcf,'WindowKeyPressFcn',@obj.keyPressCallback);

            if ~isempty(scenario.road_raw_data) && ~isempty(scenario.road_raw_data.lanelet)
                plot_lanelets(scenario.road_raw_data.lanelet,obj.scenario.options.scenario_name);
            end
            hold on
            box on
            axis equal
            xlabel('\fontsize{14}{0}$x$ [m]','Interpreter','LaTex');
            ylabel('\fontsize{14}{0}$y$ [m]','Interpreter','LaTex');
            xlim(scenario.options.plot_limits(1,:));
            ylim(scenario.options.plot_limits(2,:));
            daspect([1 1 1])
            colormap("hot"); % set colormap

            set(0,'DefaultTextFontname', 'Verdana');
            set(0,'DefaultAxesFontName', 'Verdana');

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
            hold on
        end

        function plotOnline(obj, plotting_info)
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

            priority_list = plotting_info.priorities;
            
            nObst = plotting_info.n_obstacles;
            nDynObst = plotting_info.n_dynamic_obstacles;            

            if nargin < 3
                tick_now = 1;
            end

            if isempty(plotting_info.exploration)
                plotting_info.exploration.doExploration = false;
            end

            %% Simulation state / scenario plot

            % find all the plots with property "LineWidth 1", which are different to plot_lanelets (default "LineWidth 0.5")
            % at every time step, delete all these plots while keep plot_lanelets
            h = findobj('LineWidth',1);
            delete(h)

            if plotting_info.exploration.doExploration
                visualize_exploration(plotting_info.exploration, obj.scenario.options.amount, obj.scenario.options.Hp);
            end

            if obj.plot_options.isVideoMode
                % in video mode, lanelets should be plotted at each time step
                hold on
                box on
                axis equal

                xlabel('\fontsize{14}{0}$x$ [m]','Interpreter','LaTex');
                ylabel('\fontsize{14}{0}$y$ [m]','Interpreter','LaTex');

                xlim(obj.scenario.options.plot_limits(1,:));
                ylim(obj.scenario.options.plot_limits(2,:));
                daspect([1 1 1])
                plot_lanelets(obj.scenario.road_raw_data.lanelet,obj.scenario.options.scenario_name);
                colormap("hot"); % set colormap
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
            % Sampled reference trajectory points
            for v=1:obj.nVeh
                line(   plotting_info.ref_trajectory(v,:,1), ...
                    plotting_info.ref_trajectory(v,:,2), ...
                    'Color',vehColor(priority_list(v),:),'LineStyle','none','Marker','o','MarkerFaceColor',vehColor(priority_list(v),:),'MarkerSize',3,'LineWidth',1 );
            end

            % predicted trajectory
            for v=1:obj.nVeh
                line(   plotting_info.trajectory_predictions{v}([1:obj.scenario.options.tick_per_step+1:end,end],1), ...
                    plotting_info.trajectory_predictions{v}([1:obj.scenario.options.tick_per_step+1:end,end],2), ...
                    'Color',vehColor(priority_list(v),:),'LineStyle','none','Marker','+','MarkerFaceColor',vehColor(priority_list(v),:),'MarkerSize', 3, 'LineWidth',1 );
                % Matlab R2021a:
                %'Color',vehColor(priority_list(v),:),'LineStyle','none','Marker','|','MarkerFaceColor',vehColor(priority_list(v),:),'MarkerSize', 3, 'LineWidth',1 );
                % Matlab R2020a:
                %'Color',vehColor(priority_list(v),:),'LineStyle','none','Marker','+','MarkerFaceColor',vehColor(priority_list(v),:),'MarkerSize', 3, 'LineWidth',1 );
                line(   plotting_info.trajectory_predictions{v}(:,1), ...
                    plotting_info.trajectory_predictions{v}(:,2), ...
                    'Color',vehColor(priority_list(v),:),'LineWidth',1 );
            end

            % Vehicle rectangles
            for v=1:obj.nVeh
                veh = obj.vehicles(v);
                pos_step = plotting_info.trajectory_predictions{v};
                x = pos_step(tick_now,:);
                vehiclePolygon = transformedRectangle(x(1),x(2),x(3), veh.Length,veh.Width);
                patch(   vehiclePolygon(1,:)...
                    ,vehiclePolygon(2,:)...
                    ,vehColor(priority_list(v),:)...
                    ,'LineWidth', 1 ...
                    );

                % plot the priority
                %         if obj.plot_options.isShowPriority
                %             text(x(1),x(2),num2str(result.priority(v,plotting_info.step)),'FontSize', 12, 'LineWidth',1,'Color','m');
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
                    if obj.scenario.options.bound_reachable_sets
                        text_RS = 'Bounded reachable set by lanelet boundaries';
                    else
                        text_RS = 'Unbounded reachable set';
                    end

                    if isempty(obj.plot_options.vehsReachableSets)
                        [RS_x,RS_y] = boundary(plotting_info.reachable_sets{v,obj.scenario.options.Hp});
                        line(RS_x,RS_y,'LineWidth',1.0,'Color','k');
                        text(mean(RS_x),mean(RS_y),text_RS,'LineWidth',1,'FontSize',16)
                    elseif ismember(v,obj.plot_options.vehsReachableSets)
                        % specify vehicles whose reachable sets should be shown
                        [RS_x,RS_y] = boundary(plotting_info.reachable_sets{v,obj.scenario.options.Hp});
                        line(RS_x,RS_y,'LineWidth',1.0,'Color','k');
                        text(mean(RS_x),mean(RS_y),text_RS,'LineWidth',1,'FontSize',16)
                    end

                end

                if obj.plot_options.isShowLaneletCrossingAreas
                    LCA = plotting_info.lanelet_crossing_areas{v};
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
                x0 = cellfun(@(c)c(tick_now,:), plotting_info.trajectory_predictions, 'UniformOutput', false);
                x0 = cell2mat(x0);
                if ~isempty(plotting_info.coupling_weights_reduced)
                    plot_coupling_lines(plotting_info.coupling_weights_reduced, x0, plotting_info.belonging_vector, plotting_info.coupling_info, coupling_visu)
                else
                    plot_coupling_lines(plotting_info.directed_coupling, x0, [], [], coupling_visu)
                end
            end

            %     plot distance
            %     text((iter.x0(v,1)+iter.x0(adj_v,1))/2,(iter.x0(v,2)+iter.x0(adj_v,2))/2,...
            %         num2str(round(result.distance(v,adj_v,plotting_info.step),2)),'FontSize', 12, 'LineWidth',1,'Color','b');

            % Obstacle rectangle
            for obs = 1:nObst
                patch(   plotting_info.obstacles{obs}(1,:)...
                    ,plotting_info.obstacles{obs}(2,:)...
                    ,[0.5 0.5 0.5]...
                    ,'LineWidth', 1 ...
                    );
            end

            % dynamic obstacles
            for obs = 1:nDynObst
                pos_step = plotting_info.dynamic_obstacle_fullres{obs};
                x = pos_step(tick_now,:);
                obstaclePolygon = transformedRectangle(x(1),x(2),pi/2, plotting_info.dynamic_obstacles_shape(1),plotting_info.dynamic_obstacles_shape(2));
                patch(   obstaclePolygon(1,:)...
                    ,obstaclePolygon(2,:)...
                    ,[0.5 0.5 0.5]...
                    ,'LineWidth', 1 ...
                    );
            end

            t=title(sprintf('Scenario: \\verb!%s!, Optimizer: \\verb!%s!, Strategy: \\verb!%s!, \nStep: %i, Time: %3.1fs',...
                obj.scenario.options.scenario_name,...
                'Graph Search',...
                obj.strategy,...
                plotting_info.step,...
                (plotting_info.step-1)*obj.scenario.options.dt + (tick_now-1) * obj.scenario.options.time_per_tick),'Interpreter','latex','FontSize',12);

            set(t,'HorizontalAlignment', 'center');

            drawnow
        end

        function close_figure(obj)
            close(obj.fig);
        end

        function data_queue_callback(obj, plotting_info)
            start_simulation_timer(obj);
            simulated_time = obj.scenario.options.dt * (plotting_info.step);
            simulation_time = toc(obj.timer) + obj.simulation_time_before_pause;
            pause(simulated_time - simulation_time);
            obj.plotOnline(plotting_info);
            if obj.abort
                disp('Not yet implemented');
                obj.abort = ~obj.abort;
            end
            if obj.paused
                % reset timer
                obj.timer = [];
                % save simulation 
                obj.simulation_time_before_pause = simulation_time;
                while (obj.paused)
                    %pause to allow key callback to be executed
                    pause(0.2);
                end
            end
        end

    end

    methods (Access = private)
        function start_simulation_timer(obj)
            if isempty(obj.timer)
                obj.timer=tic;
            end
        end

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
