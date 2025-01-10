classdef (Abstract) Plotter < handle

    properties
        abort (1, 1) logical % whether the simulation is to be aborted
        paused (1, 1) logical % whether the playback is paused
    end

    properties (Access = protected)
        fig matlab.ui.Figure % figure used for plotting;
        plot_options (1, 1) OptionsPlotOnline % options for plotting
        priority_colormap (:, 3) double % Colors for computation levels

        options (1, 1) Config % Config object
        scenario Scenario % (1, 1) scenario object
        vehicle_indices (1, :) int32 % indices of vehicles for which this plotter instance is responsible
        time_step (1, 1) int64 % keep track wich time step is currently plotted

        hotkey_description (1, :) string % hotkey description text to show
        number_base_hotkeys (1, 1) int32 % number of hotkeys to control Plotter class

        export_fig_config (1, 1) ExportFigConfig
        vehicle_id_radius (1, 1) double % radius of circle around vehicle ID
    end

    properties (Dependent)
        nVeh (1, 1) int32 % number of vehicles
        vehicles (1, :) Vehicle % vehicle objects

        hotkey_position (1, 1) double % position to place top left corner of hotkey description text
    end

    methods

        function result = get.nVeh(obj)
            result = length(obj.vehicle_indices);
        end

        function result = get.vehicles(obj)
            result = obj.scenario.vehicles;
        end

        function result = get.hotkey_position(obj)

            if obj.options.scenario_type == ScenarioType.commonroad
                result = diag(obj.scenario.plot_limits) + [-1.6; 0];
            elseif obj.options.scenario_type == ScenarioType.circle && obj.options.amount <= 2
                result = obj.scenario.plot_limits(:, 1) + [0; -0.25];
            elseif obj.options.scenario_type == ScenarioType.circle && obj.options.amount > 2
                result = diag(obj.scenario.plot_limits) + [-2.1; 0];
            else
                % To be defined according to the specific scenario.
                result = diag(obj.scenario.plot_limits) + [-1.6; 0];
            end

        end

        function obj = Plotter(options, scenario, vehicle_indices)
            %PLOTTER Create a Plotter object.
            %   Initialize all class members for the first time step and create a figure.
            arguments
                options (1, 1) Config
                scenario (1, 1) Scenario
                vehicle_indices (1, :) int32 = 1:options.amount
            end

            % Initialize variable for key press callback.
            obj.abort = false;

            % General plotting options
            obj.options = options;
            obj.scenario = scenario;
            obj.plot_options = options.options_plot_online;
            obj.export_fig_config = ExportFigConfig().video();
            obj.vehicle_indices = vehicle_indices;
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
                Visible = 'On', ...
                Color = [1 1 1], ...
                units = obj.export_fig_config.units, ...
                OuterPosition = [100 100 obj.export_fig_config.paperwidth obj.export_fig_config.paperheight] ...
            );

            if ~isempty(scenario.road_raw_data) && ~isempty(scenario.road_raw_data.lanelet)
                plot_lanelets(scenario.road_raw_data.lanelet);
            end

            obj.set_colormap();

            hold on
            box on
            axis equal
            xlabel('$x$ [m]', Interpreter = 'LaTex');
            ylabel('$y$ [m]', Interpreter = 'LaTex');
            xlim(obj.scenario.plot_limits(1, :));
            ylim(obj.scenario.plot_limits(2, :));
            daspect([1 1 1])
            set(0, 'DefaultTextFontname', obj.export_fig_config.fontname);
            set(0, 'DefaultAxesFontName', obj.export_fig_config.fontname);

            obj.fig.CurrentAxes.FontSize = obj.export_fig_config.fontsize;

            %
            t = text( ...
                0.5, 0.5, ...
                '20', ...
                LineWidth = 1, ...
                Color = 'black', ...
                HorizontalAlignment = 'center', ...
                FontSize = obj.export_fig_config.fontsize, ...
                Tag = "temporary" ...
            );
            extent = t.Extent;
            delete(t);
            text_width = extent(3);
            text_height = extent(4);
            % compute new radius so that text fits into circle
            obj.vehicle_id_radius = max(text_width, text_height) * 1.1/2;

            % Register key press callback function.
            set(obj.fig, 'WindowKeyPressFcn', @obj.keyPressCallback);
        end

        function delete(obj)
            obj.close_figure();
        end

        function plot(obj, plotting_info)
            %PLOT  Plot the simulation state described by a PlottingInfo object.
            arguments
                obj (1, 1) Plotter
                plotting_info (1, 1) PlottingInfo
            end

            vehicle_to_computation_level = kahn(plotting_info.directed_coupling);

            nObst = plotting_info.n_obstacles;

            %% Simulation state / scenario plot

            delete(findobj(obj.fig, Tag = "temporary"));
            delete(findobj(obj.fig, Tag = "circle"));

            if obj.plot_options.plot_priority
                % Get plot's priority colorbar and set it to visible or define a new priority colorbar.
                priority_colorbar = findobj(obj.fig, Tag = 'priority_colorbar');

                n_colors_max = size(obj.priority_colormap, 1);

                if isempty(priority_colorbar)
                    priority_colorbar = colorbar( ...
                        Tag = 'priority_colorbar' ...
                    );
                    priority_colorbar.Title.String = 'Priority';
                    priority_colorbar.TickLabels = string(1:n_colors_max);
                    priority_colorbar.TickLength = 0;
                else
                    priority_colorbar.Visible = 'on';
                end

                % Plot labels in the middle of each priority color.
                priority_colorbar.Ticks = 0.5:n_colors_max - 0.5;
                % Define the range of the colorbar according to the number of colors.
                clim([0 n_colors_max]);
            end

            %%
            for i_vehicle = obj.vehicle_indices
                % Sampled reference trajectory points
                line( ...
                    plotting_info.ref_trajectory(i_vehicle, :, 1), ...
                    plotting_info.ref_trajectory(i_vehicle, :, 2), ...
                    Color = obj.priority_colormap(vehicle_to_computation_level(i_vehicle), :), ...
                    LineStyle = 'none', ...
                    Marker = 'o', ...
                    MarkerFaceColor = obj.priority_colormap(vehicle_to_computation_level(i_vehicle), :), ...
                    MarkerSize = 3, ...
                    LineWidth = 1, ...
                    Tag = "temporary" ...
                );

                % predicted trajectory
                x = plotting_info.x0(:, i_vehicle);
                % markers at time step beginning
                line( ...
                    plotting_info.trajectory_predictions(1, :, i_vehicle), ...
                    plotting_info.trajectory_predictions(2, :, i_vehicle), ...
                    Color = obj.priority_colormap(vehicle_to_computation_level(i_vehicle), :), ...
                    LineStyle = 'none', ...
                    Marker = '+', ...
                    MarkerFaceColor = obj.priority_colormap(vehicle_to_computation_level(i_vehicle), :), ...
                    MarkerSize = 3, ...
                    LineWidth = 1, ...
                    Tag = "temporary" ...
                );
                % interpolate line with spline
                t_prediction_horizon = obj.options.dt_seconds * obj.options.Hp;
                t_sample_points = 0:obj.options.dt_seconds:t_prediction_horizon;
                dt_sampling_seconds = 50 * 1e-3;
                t_query = 0:dt_sampling_seconds:t_prediction_horizon;
                interpolated_trajectory_prediction = interp1( ...
                    t_sample_points, ...
                    [x'; squeeze(plotting_info.trajectory_predictions(:, :, i_vehicle))'], ...
                    t_query, ...
                    'spline' ...
                )';
                line( ...
                    interpolated_trajectory_prediction(1, :), ...
                    interpolated_trajectory_prediction(2, :), ...
                    Color = obj.priority_colormap(vehicle_to_computation_level(i_vehicle), :), ...
                    LineWidth = 1, ...
                    Tag = "temporary" ...
                );

                % Vehicle rectangles
                veh = obj.vehicles(i_vehicle);
                vehiclePolygon = transformed_rectangle(x(1), x(2), x(3), veh.Length, veh.Width);
                patch( ...
                    vehiclePolygon(1, :) ...
                    , vehiclePolygon(2, :) ...
                    , obj.priority_colormap(vehicle_to_computation_level(i_vehicle), :) ...
                    , LineWidth = 1 ...
                    , Tag = "temporary" ...
                );

                % plot the vehicle ID in the middle of each vehicle on a lighter background
                if obj.plot_options.plot_vehicle_id

                    rectangle( ...
                        Position = [x(1) - obj.vehicle_id_radius, x(2) - obj.vehicle_id_radius, 2 * obj.vehicle_id_radius, 2 * obj.vehicle_id_radius], ...
                        Curvature = [1, 1], ...
                        FaceColor = [1, 1, 1, 0.75], ...
                        LineStyle = 'none', ...
                        LineWidth = 1, ...
                        Tag = 'circle' ...
                    );

                    text( ...
                        x(1), x(2), ...
                        num2str(i_vehicle), ...
                        LineWidth = 1, ...
                        Color = 'black', ...
                        HorizontalAlignment = 'center', ...
                        FontSize = obj.export_fig_config.fontsize, ...
                        Tag = "temporary" ...
                    );
                end

                if obj.plot_options.plot_reachable_sets
                    obj.plot_reachable_sets(plotting_info, i_vehicle);
                end

            end

            % plot scenario adjacency
            if obj.plot_options.plot_coupling
                coupling_visu = struct( ...
                    LineWidth = 1 ...
                    , isShowLine = obj.plot_options.plot_coupling ...
                    , isShowValue = obj.plot_options.plot_weight ...
                    , radius = obj.vehicle_id_radius ...
                    , FontSize = obj.export_fig_config.fontsize - 2 ...
                );

                plot_coupling_lines( ...
                    plotting_info.weighted_coupling, ...
                    plotting_info.directed_coupling_sequential, ...
                    plotting_info.x0', ...
                    coupling_visu ...
                );

            end

            % Obstacle rectangle
            for obs = 1:nObst
                patch(plotting_info.obstacles{obs}(1, :) ...
                    , plotting_info.obstacles{obs}(2, :) ...
                    , [0.5 0.5 0.5] ...
                    , LineWidth = 1 ...
                    , Tag = "temporary" ...
                );
            end

            if obj.options.is_prioritized
                priority_text = char(obj.options.priority);
                priority_text = erase(priority_text, '_priority');
            else
                priority_text = 'centralized';
            end

            optimizer_text = lower(erase( ...
                string(obj.options.optimizer_type), ...
                ["Cpp", "Matlab"] ...
            ));

            title_text = sprintf( ...
                'Optimizer: %s, Prioritization: %s, $N_{CL}=%2i$ \nTime step: %3i, Time: %4.1f s', ...
                optimizer_text, ...
                priority_text, ...
                obj.options.max_num_CLs, ...
                plotting_info.step, ...
                plotting_info.time_seconds ...
            );
            t = title( ...
                title_text, ...
                Interpreter = 'latex' ...
            );

            set(t, HorizontalAlignment = 'center');

            drawnow
        end

        function plot_reachable_sets(obj, plotting_info, vehicle_indices)

            arguments
                obj (1, 1) Plotter
                plotting_info (1, 1) PlottingInfo
                vehicle_indices (1, :) int32 = obj.vehicle_indices;
            end

            is_plottable = ~cellfun(@isempty, plotting_info.reachable_sets);
            vehicle_indices = vehicle_indices(is_plottable);

            for i_vehicle = vehicle_indices

                if ~isempty(obj.plot_options.vehicles_reachable_sets) ...
                        && ~ismember(i_vehicle, obj.plot_options.vehicles_reachable_sets)
                    continue
                end

                line( ...
                    plotting_info.reachable_sets{i_vehicle, obj.options.Hp}(1, :), ...
                    plotting_info.reachable_sets{i_vehicle, obj.options.Hp}(2, :), ...
                    LineWidth = 1.0, ...
                    Color = 'k', ...
                    Tag = "temporary" ...
                );
            end

        end

        function fig = get_figure(obj)
            fig = obj.fig;
        end

        function close_figure(obj)
            %CLOSE_FIGURE Close the used plotting figure.
            try %#ok<TRYNC>
                close(obj.fig);
            end

        end

        function set_figure_visibility(obj, option)

            arguments
                obj Plotter
                option (1, 1) logical = true
            end

            obj.fig.Visible = option;
        end

        function set_colormap(obj, optional)
            %SET_COLORMAP Set the colormap for the plot.
            arguments
                obj (1, 1) Plotter
                optional.colormap (:, 3) double = discrete_colormap()
            end

            % Define a colormap
            obj.priority_colormap = optional.colormap;
            colormap(obj.priority_colormap);
        end

        function set_export_fig_config(obj, export_fig_config)
            arguments
                obj (1, 1) Plotter
                export_fig_config (1, 1) ExportFigConfig
            end
            obj.export_fig_config = export_fig_config;
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

            if obj.paused
                obj.hotkey_description(11) = "{\itspace}: start simulation";
            else
                obj.hotkey_description(11) = "{\itspace}: stop simulation";
            end

            delete(findobj(obj.fig, Tag = 'hotkey'));

            if obj.plot_options.plot_hotkey_description
                % Update hot key description.
                obj.plot_hotkey_description();
            end

        end

        function plot_hotkey_description(obj)
            %PLOT_HOTKEY_DESCRIPTION
            %   Plot the hotkey descriptions next to the scenario plot.

            position = obj.hotkey_position;
            text(position(1), position(2), obj.hotkey_description, ...
                HorizontalAlignment = 'left', ...
                VerticalAlignment = 'top', ...
                FontSize = obj.export_fig_config.fontsize, ...
                Tag = 'hotkey' ...
            );
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
                        find_colorbar = findall(gcf, 'Type', 'ColorBar', Tag = 'priority_colorbar');

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
