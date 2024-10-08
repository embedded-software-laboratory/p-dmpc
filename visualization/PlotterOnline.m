classdef PlotterOnline < Plotter
    %PLOTTERONLINE Plotter class for online result visualization during simulation.
    %   Along with the simulation, the different time steps can be plotted when finished by all threads.

    properties (Access = public)
        is_finished = false
    end

    properties (Access = private)
        timer % used to simulate real time plotting while receiving data from visualiuation queue
        plotting_info_collection (1, 1) struct % collect plotting info from each vehicle via data queue
        current_plotting_info PlottingInfo % (1, 1), last plotted complete plotting info

        ros2node
        subscriber

        n_finished = 0;
        is_plotting = false;
        max_time_step = -1;

    end

    methods

        function obj = PlotterOnline(options, scenario, vehicle_indices)
            %PLOTTERONLINE Construct an instance of PlotterOnline
            %   Specify the scenario and optionally the vehicle indices to plot.
            arguments
                options (1, 1) Config
                scenario (1, 1) Scenario
                vehicle_indices (1, :) int32 = 1:options.amount
            end

            obj@Plotter(options, scenario, vehicle_indices);
            obj.paused = false;
            set(obj.fig, 'WindowKeyPressFcn', @obj.keyPressCallback);

            obj.ros2node = ros2node('plotter_online');

            topic_name_subscribe = '/plotting';

            qos_config = struct( ...
                History = "keeplast", ...
                Depth = 40, ...
                Reliability = "reliable", ...
                Durability = "transientlocal" ...
            );

            if ~options.is_prioritized ...
                    || options.computation_mode == ComputationMode.sequential
                % In cenralized or sequential mode, the main function is busy
                % running the HLCs, so we plot in the callback
                callback = @obj.collect_data_and_plot;
            else
                % In parallel mode, we plot in the main function
                callback = @obj.collect_data;
            end

            obj.subscriber = ros2subscriber( ...
                obj.ros2node, ...
                topic_name_subscribe, ...
                "veh_msgs/PlottingInfo", ...
                callback, ...
                qos_config ...
            );

            if ~options.options_plot_online.is_active
                obj.set_figure_visibility(false);
            end

            obj.timer = tic;
        end

        function plot(obj, plotting_info)
            %PLOT  Plot the simulation state described by a PlottingInfo object.
            %   Plot and store the PlottingInfo object to enable replotting.
            arguments
                obj (1, 1) PlotterOnline
                plotting_info (1, 1) PlottingInfo
            end

            obj.current_plotting_info = plotting_info;
            plot@Plotter(obj, plotting_info);
        end

        function plotting_loop(obj)

            while obj.time_step <= obj.max_time_step || ~obj.is_finished

                if obj.paused
                    pause(100 * 1e-3)
                    continue
                end

                obj.plot_collected_data();
            end

            obj.close_figure();

        end

        function plot_collected_data(obj)

            % check if time step is complete (all vehicles received)
            field_name = strcat('step', num2str(obj.time_step));

            is_ready_to_plot = ...
                isfield(obj.plotting_info_collection, field_name) ...
                && nnz(~cellfun(@isempty, obj.plotting_info_collection.(field_name))) == obj.nVeh ...
                && ~obj.is_plotting;

            if ~is_ready_to_plot
                pause(20 * 1e-3)
                return
            end

            obj.is_plotting = true;
            complete_plotting_info = obj.merge_plotting_infos(obj.plotting_info_collection.(field_name));

            obj.plot(complete_plotting_info);

            %delete field
            obj.plotting_info_collection = rmfield(obj.plotting_info_collection, field_name);

            obj.time_step = obj.time_step + 1;

            % Slow down to real time
            dt = toc(obj.timer);
            time_diff = obj.options.dt_seconds - dt;

            if time_diff > 0
                pause(time_diff);
            end

            obj.timer = tic;
            obj.is_plotting = false;

        end

        function collect_data(obj, msg)

            arguments
                obj (1, 1) PlotterOnline
                msg (1, 1)
            end

            if msg.step == -1
                fprintf('Vehicle %d finished\n', msg.vehicle_indices(1));
                obj.n_finished = obj.n_finished + 1;

                if obj.n_finished == obj.nVeh
                    obj.is_finished = true;
                    disp('All vehicles finished');
                end

                return
            end

            % get plotting info from ros message
            plotting_info = obj.compute_plotting_info(msg);

            % save info
            field_name = strcat('step', num2str(plotting_info.step));
            obj.plotting_info_collection.(field_name){plotting_info.vehicle_indices(1)} = plotting_info;

            obj.max_time_step = max(obj.max_time_step, plotting_info.step);

        end

        function collect_data_and_plot(obj, msg)

            arguments
                obj (1, 1) PlotterOnline
                msg (1, 1)
            end

            obj.collect_data(msg);
            obj.plot_collected_data();

        end

    end

    methods (Access = protected)

        function plotting_info = compute_plotting_info(obj, msg)
            % compute a working plotting info object that is needed for plotting
            % input is a ros2 message
            % reconstruct matrices from list sent via ros2
            plotting_info = PlottingInfo();

            plotting_info.x0 = msg.state;
            plotting_info.trajectory_predictions = reshape( ...
                msg.trajectory_predictions, ...
                numel(msg.trajectory_predictions) / 3, ...
                3 ...
            )';
            plotting_info.ref_trajectory = zeros(1, obj.options.Hp, 2);
            plotting_info.ref_trajectory(1, :, :) = reshape(msg.ref_trajectory, numel(msg.ref_trajectory) / 2, 2);
            plotting_info.n_obstacles = msg.n_obstacles;
            plotting_info.step = msg.step;
            plotting_info.vehicle_indices = msg.vehicle_indices;
            plotting_info.time_seconds = msg.time_seconds;
            plotting_info.directed_coupling = reshape( ...
                msg.directed_coupling, ...
                obj.options.amount, ...
                obj.options.amount ...
            )';
            plotting_info.directed_coupling_sequential = reshape( ...
                msg.directed_coupling_sequential, ...
                obj.options.amount, ...
                obj.options.amount ...
            )';
            plotting_info.weighted_coupling = reshape( ...
                msg.weighted_coupling, ...
                obj.options.amount, ...
                obj.options.amount ...
            )';
        end

        function complete_plotting_info = merge_plotting_infos(obj, plotting_info_collection)
            complete_plotting_info = plotting_info_collection{1};
            complete_plotting_info.vehicle_indices = cellfun(@(x) x.vehicle_indices(1), plotting_info_collection);
            complete_plotting_info.x0 = nan( ...
                size(plotting_info_collection{1}.x0, 1), ...
                length(plotting_info_collection) ...
            );
            complete_plotting_info.trajectory_predictions = nan( ...
                size(plotting_info_collection{1}.trajectory_predictions, 1), ...
                size(plotting_info_collection{1}.trajectory_predictions, 2), ...
                length(plotting_info_collection) ...
            );
            complete_plotting_info.ref_trajectory = nan( ...
                length(plotting_info_collection), ...
                size(plotting_info_collection{1}.ref_trajectory, 2), ...
                size(plotting_info_collection{1}.ref_trajectory, 3) ...
            );

            for i = 1:length(plotting_info_collection)
                info = plotting_info_collection{i};
                complete_plotting_info.x0(:, i) = info.x0;
                complete_plotting_info.trajectory_predictions(:, :, i) = ...
                    info.trajectory_predictions;
                complete_plotting_info.ref_trajectory(i, :, :) = info.ref_trajectory(1, :, :);
            end

            n_obstacles = 0;

            for x = plotting_info_collection
                n_obstacles = n_obstacles + x{1}.n_obstacles;
            end

            complete_plotting_info.n_obstacles = n_obstacles;
            complete_plotting_info.obstacles = cellfun(@(x) x.obstacles, plotting_info_collection, UniformOutput = false);

            if obj.plot_options.plot_reachable_sets
                complete_plotting_info.reachable_sets = cellfun(@(x) x.reachable_sets, plotting_info_collection, UniformOutput = false);
            end

        end

        function keyPressCallback(obj, ~, eventdata)
            %KEY_PRESS_CALLBACK Callback function for PlotterOffline specific figure key press event.
            %   Enable/disable plotting options on hot key press and replot.
            arguments
                obj (1, 1) Plotter
                ~
                eventdata (1, 1) matlab.ui.eventdata.KeyData
            end

            keyPressCallback@Plotter(obj, [], eventdata);

            if isempty(obj.current_plotting_info.trajectory_predictions)
                return
            end

            % Replot the currently stored plotting info with possibly changed plotting options.
            obj.plot(obj.current_plotting_info);
        end

    end

end
