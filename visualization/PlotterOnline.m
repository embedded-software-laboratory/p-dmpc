classdef PlotterOnline < handle
    properties
        abort
        paused
    end
    properties (Access = private)
        fig
        resolution
        plot_options % struct to store logical variables indicating whether to show vehicle ID/priority/coupling/coupling weights
        scenario
        strategy
        vehicles
        nVeh
        veh_indices % indices of vehicles for which this plotter instance is responsible
        timer % used to simulate real time plotting while receiving data from visualiuation queue
        simulation_time_offset % used to backup timer when visualization is paused
        time_step % keep track wich time step should be plotted next
        plotting_info_collection % collect plotting info from each vehicle via data queue
    end
    methods
        function obj = PlotterOnline(scenario, veh_indices)
            % variables for key press callback
            obj.paused = false;
            obj.abort = false;
            obj.resolution = [1920 1080];
            obj.plot_options = scenario.options.options_plot_online;
            obj.scenario = scenario;
            obj.time_step = 1;
            obj.strategy = HLCFactory.get_controller_name(obj.scenario.options);
            obj.vehicles = scenario.vehicles;
            if nargin <= 1
                obj.veh_indices = 1:scenario.options.amount;
                obj.nVeh = scenario.options.amount;

            else
                obj.veh_indices = veh_indices;
                obj.nVeh = length(veh_indices);
            end
            if obj.nVeh == 1
                % deactivate coupling lines for dist. plotting
                obj.plot_options.plot_coupling = 0;
            end

            obj.simulation_time_offset = 0 ;
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

            set(0,'DefaultTextFontname', 'Verdana');
            set(0,'DefaultAxesFontName', 'Verdana');
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
            %   visu: struct with fields: 'plot_vehicle_id', 'plot_priority', 'plot_coupling' and 'plot_weight'
            %

            priority_list = plotting_info.priorities;

            nObst = plotting_info.n_obstacles;
            nDynObst = plotting_info.n_dynamic_obstacles;

            if nargin < 3
                tick_now = 1;
            end

            %% Simulation state / scenario plot

            % find all the plots with property "LineWidth 1", which are different to plot_lanelets (default "LineWidth 0.5")
            % at every time step, delete all these plots while keep plot_lanelets
            h = findobj('LineWidth', 1);
            delete(h);

            if obj.plot_options.is_video_mode
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
            end

            % Define a new colormap in the first timestep, else get the colormap already associated with the plot.
            if plotting_info.step == 1
                [priority_colormap, n_colors_max] = discrete_colormap();
                colormap(priority_colormap);
            else
                priority_colormap = get(gcf, 'Colormap');
                n_colors_max = size(priority_colormap, 1);
            end

            find_text_hotkey = findobj('Tag','hotkey');
            if obj.plot_options.plot_hotkey_description
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
                    if strcmp(obj.scenario.options.scenario_name,'Commonroad')
                        x_text_hotkey = obj.scenario.options.plot_limits(1,1) - 1.5;
                        y_text_hotkey = obj.scenario.options.plot_limits(2,2) - 0.5;
                    elseif strcmp(obj.scenario.options.scenario_name,'Circle_scenario')
                        x_text_hotkey = obj.scenario.options.plot_limits(1,1) - 2.0;
                        y_text_hotkey = obj.scenario.options.plot_limits(2,2) - 0.5;
                    else
                        % to be define according to the specific scenario
                        x_text_hotkey = obj.scenario.options.plot_limits(1,1) - 1.5;
                        y_text_hotkey = obj.scenario.options.plot_limits(2,2) - 0.5;
                    end
                    text(x_text_hotkey, y_text_hotkey, HotkeyDesc, 'FontSize',12, 'Tag','hotkey');
                end
            else
                % remove hot keys description if it was painted
                delete(find_text_hotkey);
            end

            if obj.plot_options.plot_priority
                % Get plot's priority colorbar and set it to visible or define a new priority colorbar.
                priority_colorbar = findobj('Tag','priority_colorbar');
                if isempty(priority_colorbar)
                    priority_colorbar = colorbar('Tag','priority_colorbar','FontName','Verdana','FontSize',9);
                    priority_colorbar.Title.String = 'Priority';
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
            % Sampled reference trajectory points
            for v=obj.veh_indices
                line(   plotting_info.ref_trajectory(v,:,1), ...
                    plotting_info.ref_trajectory(v,:,2), ...
                    'Color',priority_colormap(priority_list(v),:),'LineStyle','none','Marker','o', ...
                    'MarkerFaceColor',priority_colormap(priority_list(v),:),'MarkerSize',3,'LineWidth',1 );
            end

            % predicted trajectory
            for v=obj.veh_indices
                line(   plotting_info.trajectory_predictions{v}([1:obj.scenario.options.tick_per_step+1:end,end],1), ...
                    plotting_info.trajectory_predictions{v}([1:obj.scenario.options.tick_per_step+1:end,end],2), ...
                    'Color',priority_colormap(priority_list(v),:),'LineStyle','none','Marker','+', ...
                    'MarkerFaceColor',priority_colormap(priority_list(v),:),'MarkerSize', 3, 'LineWidth',1 );
                % Matlab R2021a:
                %'Color',priority_colormap(priority_list(v),:),'LineStyle','none','Marker','|','MarkerFaceColor',priority_colormap(priority_list(v),:),'MarkerSize', 3, 'LineWidth',1 );
                % Matlab R2020a:
                %'Color',priority_colormap(priority_list(v),:),'LineStyle','none','Marker','+','MarkerFaceColor',priority_colormap(priority_list(v),:),'MarkerSize', 3, 'LineWidth',1 );
                line(   plotting_info.trajectory_predictions{v}(:,1), ...
                    plotting_info.trajectory_predictions{v}(:,2), ...
                    'Color',priority_colormap(priority_list(v),:),'LineWidth',1 );
            end

            % Vehicle rectangles
            for v=obj.veh_indices
                veh = obj.vehicles(v);
                pos_step = plotting_info.trajectory_predictions{v};
                x = pos_step(tick_now,:);
                vehiclePolygon = transformedRectangle(x(1),x(2),x(3), veh.Length,veh.Width);
                patch(   vehiclePolygon(1,:)...
                    ,vehiclePolygon(2,:)...
                    ,priority_colormap(priority_list(v),:)...
                    ,'LineWidth', 1 ...
                    );

                % plot the priority
                %         if obj.plot_options.plot_priority
                %             text(x(1),x(2),num2str(result.priority(v,plotting_info.step)),'FontSize', 12, 'LineWidth',1,'Color','m');
                %         end

                % plot the vehicle index in the middle of each vehicle on a lighter background
                if obj.plot_options.plot_vehicle_id
                    radius = veh.Width * 0.95 / 2;
                    rectangle('Position', [x(1)-radius, x(2)-radius, 2*radius, 2*radius], 'Curvature', [1,1], ...
                        'FaceColor', [1, 1, 1, 0.75], 'LineStyle', 'none', 'LineWidth', 1, 'Tag', 'circle');
                    text(x(1), x(2), num2str(v),'FontSize', 10, 'LineWidth', 1, 'Color', 'black', 'HorizontalAlignment', 'center');
                end

                % plot the vehicle ID
                %         if obj.plot_options.plot_vehicle_id
                %             text(x(1)+0.1,x(2)+0.1,num2str(veh.ID),'FontSize', 12, 'LineWidth',1,'Color','b');
                %         end

                if obj.plot_options.plot_reachable_sets
                    if obj.scenario.options.bound_reachable_sets
                        text_RS = 'Bounded reachable set by lanelet boundaries';
                    else
                        text_RS = 'Unbounded reachable set';
                    end

                    if isempty(obj.plot_options.vehicles_reachable_sets)
                        [RS_x,RS_y] = boundary(plotting_info.reachable_sets{v,obj.scenario.options.Hp});
                        line(RS_x,RS_y,'LineWidth',1.0,'Color','k');
                        text(mean(RS_x),mean(RS_y),text_RS,'LineWidth',1,'FontSize',16)
                    elseif ismember(v,obj.plot_options.vehicles_reachable_sets)
                        % specify vehicles whose reachable sets should be shown
                        [RS_x,RS_y] = boundary(plotting_info.reachable_sets{v,obj.scenario.options.Hp});
                        line(RS_x,RS_y,'LineWidth',1.0,'Color','k');
                        text(mean(RS_x),mean(RS_y),text_RS,'LineWidth',1,'FontSize',16)
                    end

                end

                if obj.plot_options.plot_lanelet_crossing_areaas && ~isempty(plotting_info.lanelet_crossing_areas)
                    LCA = plotting_info.lanelet_crossing_areas{v};
                    if ~isempty(LCA)
                        if isempty(obj.plot_options.vehicles_lanelet_crossing_areas)
                            LCAs_xy = [LCA{:}];
                            line(LCAs_xy(1,:),LCAs_xy(2,:),'LineWidth',1.0,'Color','k');
                        elseif ismember(v,obj.plot_options.vehicles_lanelet_crossing_areas)
                            % specify vehicles whose lanelet crossing areas should be shown
                            LCAs_xy = [LCA{:}];
                            line(LCAs_xy(1,:),LCAs_xy(2,:),'LineWidth',1.0,'Color','k');
                        end
                        text(max(LCAs_xy(1,:))+0.02,max(LCAs_xy(2,:)),'Lanelet crossing area','LineWidth',2,'FontSize',16)
                    end
                end
            end

            % plot scenario adjacency
            if obj.plot_options.plot_coupling
                coupling_visu = struct('FontSize',9,'LineWidth',1,'isShowLine',obj.plot_options.plot_coupling,'isShowValue',obj.plot_options.plot_weight, 'radius', radius);
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

        function fig = get_figure(obj)
            fig = obj.fig;
        end

        function clear_figure(obj)
            clf(obj.fig);
        end

        function close_figure(obj)
            close(obj.fig);
        end

        function set_figure_visibility(obj, option)
            arguments
                obj PlotterOnline
                option (1,:) char {mustBeMember(option,{'On','Off'})} = 'On'
            end
            obj.fig.Visible = option;
        end

        function data_queue_callback(obj, plotting_info)
            % TODO What to do if message is lost? timeout per plotting timestep?
            % save info
            field_name = strcat('step',num2str(plotting_info.step));
            obj.plotting_info_collection.(field_name){plotting_info.veh_indices(1)} = plotting_info;

            % check if time step is complete (all vehicles received)
            field_name = strcat('step',num2str(obj.time_step));
            if length(obj.plotting_info_collection.(field_name)) == obj.nVeh
                complete = true;
                for i = 1:length(obj.plotting_info_collection.(field_name))
                    if isempty(obj.plotting_info_collection.(field_name){i})
                        complete = false;
                        break;
                    end
                end
                if complete
                    complete_plotting_info = obj.merge_plotting_infos(obj.plotting_info_collection.(field_name));
                    start_simulation_timer(obj);
                    simulated_time = obj.scenario.options.dt * (complete_plotting_info.step);
                    simulation_time = toc(obj.timer) + obj.simulation_time_offset;
                    time_diff = simulated_time - simulation_time;
                    % avoid plotter trying catching up when simulation is
                    % slow
                    if time_diff < 0
                        obj.simulation_time_offset = obj.simulation_time_offset + time_diff;
                    else
                        pause(time_diff);
                    end
                    obj.plotOnline(complete_plotting_info);

                    %delete field
                    obj.plotting_info_collection = rmfield(obj.plotting_info_collection,field_name);
                    obj.time_step = obj.time_step + 1;
                end
            end            
            
            if obj.abort
                disp('Aborting experiment not yet implemented');
                obj.abort = ~obj.abort;
            end
            if obj.paused
                % reset timer
                obj.timer = [];
                % save simulation
                obj.simulation_time_offset = simulation_time;
                while (obj.paused)
                    %pause to allow key callback to be executed
                    pause(0.1);
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

        function complete_plotting_info = merge_plotting_infos(obj, plotting_info_collection)
            complete_plotting_info = plotting_info_collection{1};
            complete_plotting_info.veh_indices = cellfun(@(x) x.veh_indices(1), plotting_info_collection);
            for i = 1:length(plotting_info_collection)
                info = plotting_info_collection{i};
                trajectory_predictions{i,1} = info.trajectory_predictions;
                ref_trajectory(i,:,:) = info.ref_trajectory(1,:,:);
            end
            complete_plotting_info.trajectory_predictions = trajectory_predictions;
            complete_plotting_info.ref_trajectory = ref_trajectory;
            complete_plotting_info.priorities = cellfun(@(x) x.priorities, plotting_info_collection)';
            n_obstacles = 0;
            for x = plotting_info_collection
                n_obstacles = n_obstacles + x{1}.n_obstacles;
            end
            complete_plotting_info.n_obstacles = n_obstacles;
            complete_plotting_info.obstacles = cellfun(@(x) x.obstacles, plotting_info_collection, 'UniformOutput', false);
            n_dynamic_obstacles = 0;
            for x = plotting_info_collection
                n_dynamic_obstacles = n_dynamic_obstacles + x{1}.n_dynamic_obstacles;
            end
            complete_plotting_info.n_dynamic_obstacles = n_dynamic_obstacles;
            complete_plotting_info.dynamic_obstacles = cellfun(@(x) x.dynamic_obstacles, plotting_info_collection, 'UniformOutput', false);
            complete_plotting_info.dynamic_obstacles_shape = cellfun(@(x) x.dynamic_obstacles_shape, plotting_info_collection, 'UniformOutput', false);
            if obj.plot_options.plot_reachable_sets
                for i = 1:length(plotting_info_collection)
                    info = plotting_info_collection{i};
                    complete_plotting_info.reachable_sets{i,:} = info.reachable_sets;
                end
            end
            if obj.plot_options.plot_lanelet_crossing_areaas
                complete_plotting_info.lanelet_crossing_areas = cellfun(@(x) x.lanelet_crossing_areas, plotting_info_collection, 'UniformOutput', false);
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
                    obj.plot_options.plot_vehicle_id = ~obj.plot_options.plot_vehicle_id;
                    if obj.plot_options.plot_vehicle_id
                        disp('Show vehicle.')
                    else
                        disp('Hide Vehicle IDs.')
                    end
                case 'p'
                    obj.plot_options.plot_priority = ~obj.plot_options.plot_priority;
                    if obj.plot_options.plot_priority
                        disp('Show vehicle priorities.')
                    else
                        disp('Hide vehicle priorities.')
                        find_colorbar = findall(gcf,'Type','ColorBar','Tag','priority_colorbar');
                        if ~isempty(find_colorbar)
                            find_colorbar.Visible = 'off';
                        end
                    end
                case 'c'
                    if obj.nVeh == 1
                        disp('Coupling lines not supported for distributed plotting')
                    else
                        obj.plot_options.plot_coupling = ~obj.plot_options.plot_coupling;
                        if obj.plot_options.plot_coupling
                            disp('Show couplings lines.')
                        else
                            disp('Hide couplings lines.')
                        end
                    end
                case 'w'
                    obj.plot_options.plot_weight = ~obj.plot_options.plot_weight;
                    if obj.plot_options.plot_weight
                        disp('Show couplings weights.')
                    else
                        disp('Hide couplings weights.')
                    end
                case 'h'
                    obj.plot_options.plot_hotkey_description = ~obj.plot_options.plot_hotkey_description;
                    if obj.plot_options.plot_hotkey_description
                        disp('Show hot key descriptions.')
                    else
                        disp('Hide hot key descriptions.')
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
