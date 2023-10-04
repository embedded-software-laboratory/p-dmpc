classdef (Abstract) Plotter < handle

    properties
        abort (1, 1) logical % whether the simulation is to be aborted
        paused (1, 1) logical % whether the playback is paused
    end

    properties (Access = protected)
        fig (1, 1) matlab.ui.Figure % figure used for plotting
        plot_options (1, 1) OptionsPlotOnline % options for plotting
        resolution (1, 2) int32 % pixels to plot in horizontal and vertical direction
        priority_colormap (:, 3) double % Colors for computation levels

        scenario (1, 1) Scenario % scenario object
        veh_indices (1, :) int32 % indices of vehicles for which this plotter instance is responsible
        time_step (1, 1) int64 % keep track wich time step is currently plotted

        hotkey_description (1, :) string % hotkey description text to show
        number_base_hotkeys (1, 1) int32 % number of hotkeys to control Plotter class
    end

    properties (Dependent)
        nVeh (1, 1) int32 % number of vehicles
        strategy (1, 1) string % controller strategy to print
        vehicles (1, :) Vehicle % vehicle objects

        hotkey_position (1, 1) double % position to place top left corner of hotkey description text
    end

    methods

        function result = get.nVeh(obj)
            result = length(obj.veh_indices);
        end

        function result = get.strategy(obj)
            result = HLCFactory.get_controller_name(obj.scenario.options);
        end

        function result = get.vehicles(obj)
            result = obj.scenario.vehicles;
        end

        function result = get.hotkey_position(obj)

            if obj.scenario.options.scenario_type == ScenarioType.commonroad
                result = diag(obj.scenario.options.plot_limits) + [-1.6; 0];
            elseif obj.scenario.options.scenario_type == ScenarioType.circle && obj.scenario.options.amount <= 2
                result = obj.scenario.options.plot_limits(:, 1) + [0; -0.25];
            elseif obj.scenario.options.scenario_type == ScenarioType.circle && obj.scenario.options.amount > 2
                result = diag(obj.scenario.options.plot_limits) + [-2.1; 0];
            else
                % To be defined according to the specific scenario.
                result = diag(obj.scenario.options.plot_limits) + [-1.6; 0];
            end

        end

        function obj = Plotter(scenario, veh_indices)
            %PLOTTER Create a Plotter object.
            %   Initialize all class members for the first time step and create a figure.
            arguments
                scenario (1, 1) Scenario
                veh_indices (1, :) int32 = 1:scenario.options.amount
            end

            % Initialize variable for key press callback.
            obj.abort = false;

            % General plotting options.
            obj.resolution = [1920 1080];
            obj.plot_options = scenario.options.options_plot_online;
            obj.scenario = scenario;
            obj.veh_indices = veh_indices;
            obj.time_step = 1;
            % Deactivate coupling lines for distributed plotting.
            if obj.nVeh == 1
                obj.plot_options.plot_coupling = false;
            end

            % Initialize hotkey description.
            obj.hotkey_description(1) = 'Hotkey:';
            obj.hotkey_description(2) = '';
            obj.hotkey_description(9) = '{\ith}: hide hot key descriptions';
            obj.hotkey_description(10) = '';
            obj.hotkey_description(12) = "{\itesc}: abort";
            obj.number_base_hotkeys = 12;
            obj.update_hotkey_description(true);

            % Create figure.
            obj.fig = figure( ...
                'Visible', 'On', ...
                'Color', [1 1 1], ...
                'units', 'pixel', ...
                'OuterPosition', [100 100 obj.resolution(1) obj.resolution(2)] ...
            );

            if ~isempty(scenario.road_raw_data) && ~isempty(scenario.road_raw_data.lanelet)
                plot_lanelets(scenario.road_raw_data.lanelet, obj.scenario.options.scenario_type);
            end

            % Define a colormap
            [obj.priority_colormap, ~] = discrete_colormap();
            colormap(obj.priority_colormap);

            hold on
            box on
            axis equal
            xlabel('\fontsize{14}{0}$x$ [m]', 'Interpreter', 'LaTex');
            ylabel('\fontsize{14}{0}$y$ [m]', 'Interpreter', 'LaTex');
            xlim(scenario.options.plot_limits(1, :));
            ylim(scenario.options.plot_limits(2, :));
            daspect([1 1 1])
            set(0, 'DefaultTextFontname', 'Verdana');
            set(0, 'DefaultAxesFontName', 'Verdana');

            % Register key press callback function.
            set(obj.fig, 'WindowKeyPressFcn', @obj.keyPressCallback);
        end

        function plot(obj, plotting_info)
            %PLOT  Plot the simulation state described by a PlottingInfo object.
            arguments
                obj (1, 1) Plotter
                plotting_info (1, 1) PlottingInfo
            end

            priority_list = plotting_info.priorities;

            nObst = plotting_info.n_obstacles;
            nDynObst = plotting_info.n_dynamic_obstacles;

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

                xlabel('\fontsize{14}{0}$x$ [m]', 'Interpreter', 'LaTex');
                ylabel('\fontsize{14}{0}$y$ [m]', 'Interpreter', 'LaTex');

                xlim(obj.scenario.options.plot_limits(1, :));
                ylim(obj.scenario.options.plot_limits(2, :));
                daspect([1 1 1])

                if (obj.scenario.options.scenario_type == ScenarioType.lanelet2) | (obj.scenario.options.scenario_type == ScenarioType.commonroad)
                    plot_lanelets(obj.scenario.road_raw_data.lanelet, obj.scenario.options.scenario_type);
                end

            end

            find_text_hotkey = findobj('Tag', 'hotkey');

            if obj.plot_options.plot_hotkey_description
                % Update hot key description.
                delete(find_text_hotkey);
                obj.plot_hotkey_description();
            else
                % Remove hot key description if it was painted and not to be shown anymore.
                delete(find_text_hotkey);
            end

            if obj.plot_options.plot_priority
                % Get plot's priority colorbar and set it to visible or define a new priority colorbar.
                priority_colorbar = findobj('Tag', 'priority_colorbar');

                n_colors_max = size(obj.priority_colormap, 1);

                if isempty(priority_colorbar)
                    priority_colorbar = colorbar('Tag', 'priority_colorbar', 'FontName', 'Verdana', 'FontSize', 9);
                    priority_colorbar.Title.String = 'Priority';
                    priority_colorbar.TickLabels = string(1:n_colors_max);
                    priority_colorbar.TickLength = 0;
                else
                    priority_colorbar.Visible = 'on';
                end

                % Plot labels in the middle of each priority color.
                priority_colorbar.Ticks = 0.5:n_colors_max - 0.5;
                % Define the range of the colorbar according to the number of colors.
                caxis([0 n_colors_max]);
                %         clim([0 n_colors_max]) % renamed from caxis in R2022a
            end

            %%
            % Sampled reference trajectory points
            for v = obj.veh_indices
                line(plotting_info.ref_trajectory(v, :, 1), ...
                    plotting_info.ref_trajectory(v, :, 2), ...
                    'Color', obj.priority_colormap(priority_list(v), :), 'LineStyle', 'none', 'Marker', 'o', ...
                    'MarkerFaceColor', obj.priority_colormap(priority_list(v), :), 'MarkerSize', 3, 'LineWidth', 1);
            end

            % predicted trajectory
            for v = obj.veh_indices
                line(plotting_info.trajectory_predictions{v}([1:obj.scenario.options.tick_per_step + 1:end, end], 1), ...
                    plotting_info.trajectory_predictions{v}([1:obj.scenario.options.tick_per_step + 1:end, end], 2), ...
                    'Color', obj.priority_colormap(priority_list(v), :), 'LineStyle', 'none', 'Marker', '+', ...
                    'MarkerFaceColor', obj.priority_colormap(priority_list(v), :), 'MarkerSize', 3, 'LineWidth', 1);
                % Matlab R2021a:
                %'Color',priority_colormap(priority_list(v),:),'LineStyle','none','Marker','|','MarkerFaceColor',priority_colormap(priority_list(v),:),'MarkerSize', 3, 'LineWidth',1 );
                % Matlab R2020a:
                %'Color',priority_colormap(priority_list(v),:),'LineStyle','none','Marker','+','MarkerFaceColor',priority_colormap(priority_list(v),:),'MarkerSize', 3, 'LineWidth',1 );
                line(plotting_info.trajectory_predictions{v}(:, 1), ...
                    plotting_info.trajectory_predictions{v}(:, 2), ...
                    'Color', obj.priority_colormap(priority_list(v), :), 'LineWidth', 1);
            end

            % Vehicle rectangles
            for v = obj.veh_indices
                veh = obj.vehicles(v);
                pos_step = plotting_info.trajectory_predictions{v};
                x = pos_step(plotting_info.tick_now, :);
                vehiclePolygon = transformed_rectangle(x(1), x(2), x(3), veh.Length, veh.Width);
                patch(vehiclePolygon(1, :) ...
                    , vehiclePolygon(2, :) ...
                    , obj.priority_colormap(priority_list(v), :) ...
                    , 'LineWidth', 1 ...
                );

                % plot the priority
                %         if obj.plot_options.plot_priority
                %             text(x(1),x(2),num2str(result.priority(v,plotting_info.step)),'FontSize', 12, 'LineWidth',1,'Color','m');
                %         end

                % plot the vehicle index in the middle of each vehicle on a lighter background
                if obj.plot_options.plot_vehicle_id
                    radius = veh.Width * 0.95/2;
                    rectangle('Position', [x(1) - radius, x(2) - radius, 2 * radius, 2 * radius], 'Curvature', [1, 1], ...
                        'FaceColor', [1, 1, 1, 0.75], 'LineStyle', 'none', 'LineWidth', 1, 'Tag', 'circle');
                    text(x(1), x(2), num2str(v), 'FontSize', 10, 'LineWidth', 1, 'Color', 'black', 'HorizontalAlignment', 'center');
                end

                % plot the vehicle ID
                %         if obj.plot_options.plot_vehicle_id
                %             text(x(1)+0.1,x(2)+0.1,num2str(veh.ID),'FontSize', 12, 'LineWidth',1,'Color','b');
                %         end

                if obj.plot_options.plot_reachable_sets

                    if isempty(obj.plot_options.vehicles_reachable_sets)
                        [RS_x, RS_y] = boundary(plotting_info.reachable_sets{v, obj.scenario.options.Hp});
                        line(RS_x, RS_y, 'LineWidth', 1.0, 'Color', 'k');
                    elseif ismember(v, obj.plot_options.vehicles_reachable_sets)
                        % specify vehicles whose reachable sets should be shown
                        [RS_x, RS_y] = boundary(plotting_info.reachable_sets{v, obj.scenario.options.Hp});
                        line(RS_x, RS_y, 'LineWidth', 1.0, 'Color', 'k');
                    end

                end

                if obj.plot_options.plot_lanelet_crossing_areas && ~isempty(plotting_info.lanelet_crossing_areas)
                    LCA = plotting_info.lanelet_crossing_areas{v};

                    if ~isempty(LCA)

                        if isempty(obj.plot_options.vehicles_lanelet_crossing_areas)
                            LCAs_xy = [LCA{:}];
                            line(LCAs_xy(1, :), LCAs_xy(2, :), 'LineWidth', 1, 'Color', 'k');
                        elseif ismember(v, obj.plot_options.vehicles_lanelet_crossing_areas)
                            % specify vehicles whose lanelet crossing areas should be shown
                            LCAs_xy = [LCA{:}];
                            line(LCAs_xy(1, :), LCAs_xy(2, :), 'LineWidth', 1, 'Color', 'k');
                        end

                    end

                end

            end

            % plot scenario adjacency
            if obj.plot_options.plot_coupling
                radius = veh.Width * 0.95/2;
                coupling_visu = struct('FontSize', 9, 'LineWidth', 1, 'isShowLine', obj.plot_options.plot_coupling, 'isShowValue', obj.plot_options.plot_weight, 'radius', radius);
                x0 = cellfun(@(c)c(plotting_info.tick_now, :), plotting_info.trajectory_predictions, 'UniformOutput', false);
                x0 = cell2mat(x0);

                if ~isempty(plotting_info.weighted_coupling_reduced)
                    plot_coupling_lines(plotting_info.weighted_coupling_reduced, x0, plotting_info.belonging_vector, plotting_info.is_virtual_obstacle, coupling_visu)
                else
                    plot_coupling_lines(plotting_info.directed_coupling, x0, [], [], coupling_visu)
                end

            end

            %     plot distance
            %     text((iter.x0(v,1)+iter.x0(adj_v,1))/2,(iter.x0(v,2)+iter.x0(adj_v,2))/2,...
            %         num2str(round(result.distance(v,adj_v,plotting_info.step),2)),'FontSize', 12, 'LineWidth',1,'Color','b');

            % Obstacle rectangle
            for obs = 1:nObst
                patch(plotting_info.obstacles{obs}(1, :) ...
                    , plotting_info.obstacles{obs}(2, :) ...
                    , [0.5 0.5 0.5] ...
                    , 'LineWidth', 1 ...
                );
            end

            % dynamic obstacles
            for obs = 1:nDynObst
                pos_step = plotting_info.dynamic_obstacle_fullres{obs};
                x = pos_step(plotting_info.tick_now, :);
                obstaclePolygon = transformed_rectangle(x(1), x(2), pi / 2, plotting_info.dynamic_obstacles_shape(1), plotting_info.dynamic_obstacles_shape(2));
                patch(obstaclePolygon(1, :) ...
                    , obstaclePolygon(2, :) ...
                    , [0.5 0.5 0.5] ...
                    , 'LineWidth', 1 ...
                );
            end

            t = title(sprintf('Scenario: \\verb!%s!, Optimizer: \\verb!%s!, Strategy: \\verb!%s!, \nStep: %i, Time: %3.1fs', ...
                obj.scenario.options.scenario_type, ...
                'Graph Search', ...
                obj.strategy, ...
                plotting_info.step, ...
                (double(plotting_info.step) - 1) * obj.scenario.options.dt_seconds + (double(plotting_info.tick_now) - 1) * obj.scenario.options.time_per_tick), 'Interpreter', 'latex', 'FontSize', 12);

            set(t, 'HorizontalAlignment', 'center');

            drawnow
        end

        function fig = get_figure(obj)
            fig = obj.fig;
        end

        function close_figure(obj)
            %CLOSE_FIGURE Close the used plotting figure.

            close(obj.fig);
        end

    end

    methods (Access = protected)

        function update_hotkey_description(obj, ~)
            %UPDATE_HOTKEY_DESCRIPTION
            %   Set the hotkey descriptions depending on if the hot key is currently active or inactive.

            if obj.plot_options.plot_priority
                obj.hotkey_description(3) = '{\itp}: hide priority colorbar';
            else
                obj.hotkey_description(3) = '{\itp}: show priority colorbar';
            end

            if obj.plot_options.plot_vehicle_id
                obj.hotkey_description(4) = '{\iti}: hide vehicle IDs';
            else
                obj.hotkey_description(4) = '{\iti}: show vehicle IDs';
            end

            if obj.plot_options.plot_coupling
                obj.hotkey_description(5) = '{\itc}: hide coupling lines';
            else
                obj.hotkey_description(5) = '{\itc}: show coupling lines';
            end

            if obj.plot_options.plot_weight
                obj.hotkey_description(6) = '{\itw}: hide coupling weights';
            else
                obj.hotkey_description(6) = '{\itw}: show coupling weights';
            end

            if obj.plot_options.plot_reachable_sets
                obj.hotkey_description(7) = '{\itr}: hide reachable sets';
            else
                obj.hotkey_description(7) = '{\itr}: show reachable sets';
            end

            if obj.plot_options.plot_lanelet_crossing_areas
                obj.hotkey_description(8) = '{\itl}: hide lanelet crossing areas';
            else
                obj.hotkey_description(8) = '{\itl}: show lanelet crossing areas';
            end

            if obj.paused
                obj.hotkey_description(11) = "{\itspace}: start simulation";
            else
                obj.hotkey_description(11) = "{\itspace}: stop simulation";
            end

        end

        function plot_hotkey_description(obj)
            %PLOT_HOTKEY_DESCRIPTION
            %   Plot the hotkey descriptions next to the scenario plot.

            position = obj.hotkey_position;
            text(position(1), position(2), obj.hotkey_description, 'FontSize', 12, ...
                'HorizontalAlignment', 'left', 'VerticalAlignment', 'top', 'Tag', 'hotkey');
        end

        function keyPressCallback(obj, ~, eventdata)
            %KEY_PRESS_CALLBACK Callback function for Plotter figure key press event.
            %   Enable/disable plotting options on hot key press and update the hot key description.
            arguments
                obj (1, 1) Plotter
                ~
                eventdata (1, 1) matlab.ui.eventdata.KeyData
            end

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
                        find_colorbar = findall(gcf, 'Type', 'ColorBar', 'Tag', 'priority_colorbar');

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

                case 'r'
                    obj.plot_options.plot_reachable_sets = ~obj.plot_options.plot_reachable_sets;

                    if obj.plot_options.plot_reachable_sets
                        disp('Show reachable sets.')
                    else
                        disp('Hide reachable sets.')
                    end

                case 'l'
                    obj.plot_options.plot_lanelet_crossing_areas = ~obj.plot_options.plot_lanelet_crossing_areas;

                    if obj.plot_options.plot_lanelet_crossing_areas
                        disp('Show lanelet crossing areas.')
                    else
                        disp('Hide lanelet crossing areas.')
                    end

                case 'h'
                    obj.plot_options.plot_hotkey_description = ~obj.plot_options.plot_hotkey_description;

                    if obj.plot_options.plot_hotkey_description
                        disp('Show hot key descriptions.')
                    else
                        disp('Hide hot key descriptions.')
                    end

            end

            obj.update_hotkey_description();
        end

    end

end
