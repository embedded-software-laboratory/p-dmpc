classdef PlotterOnline < Plotter
    %PLOTTERONLINE Plotter class for online result visualization during simulation.
    %   Along with the simulation, the different time steps can be plotted when finished by all threads.

    properties (Access = private)
        timer % used to simulate real time plotting while receiving data from visualiuation queue
        simulation_time_offset (1, 1) double % used to backup timer when visualization is paused
        plotting_info_collection (1, 1) struct % collect plotting info from each vehicle via data queue
        current_plotting_info (1, 1) PlottingInfo % last plotted complete plotting info
    end

    methods

        function obj = PlotterOnline(scenario, veh_indices)
            %PLOTTERONLINE Construct an instance of PlotterOnline
            %   Specify the scenario and optionally the vehicle indices to plot.
            arguments
                scenario (1, 1) Scenario
                veh_indices (1, :) int32 = 1:scenario.options.amount
            end

            obj@Plotter(scenario, veh_indices);
            obj.paused = false;
            obj.simulation_time_offset = 0;
            set(obj.fig, 'WindowKeyPressFcn', @obj.keyPressCallback);
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

        function set_figure_visibility(obj, option)

            arguments
                obj PlotterOnline
                option (1, 1) logical = true
            end

            obj.fig.Visible = option;
        end

        function data_queue_callback(obj, plotting_info)

            arguments
                obj (1, 1) PlotterOnline
                plotting_info (1, 1) PlottingInfo
            end

            % TODO What to do if message is lost? timeout per plotting timestep?
            % save info
            field_name = strcat('step', num2str(plotting_info.step));
            obj.plotting_info_collection.(field_name){plotting_info.veh_indices(1)} = plotting_info;

            % check if time step is complete (all vehicles received)
            field_name = strcat('step', num2str(obj.time_step));

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
                    simulated_time = obj.scenario.options.dt_seconds * (complete_plotting_info.step);
                    simulation_time = toc(obj.timer) + obj.simulation_time_offset;
                    time_diff = simulated_time - simulation_time;
                    % avoid plotter trying catching up when simulation is
                    % slow
                    if time_diff < 0
                        obj.simulation_time_offset = obj.simulation_time_offset + time_diff;
                    else
                        pause(time_diff);
                    end

                    obj.plot(complete_plotting_info);

                    %delete field
                    obj.plotting_info_collection = rmfield(obj.plotting_info_collection, field_name);
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

        function ros2_callback(obj, msg)

            arguments
                obj (1, 1) PlotterOnline
                msg (1, 1)
            end

            % get plotting info from ros message
            plotting_info = obj.compute_plotting_info(msg);

            % save info
            field_name = strcat('step', num2str(plotting_info.step));
            obj.plotting_info_collection.(field_name){plotting_info.veh_indices(1)} = plotting_info;

            % check if time step is complete (all vehicles received)
            field_name = strcat('step', num2str(obj.time_step));

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
                    simulated_time = obj.scenario.options.dt_seconds * (complete_plotting_info.step);
                    simulation_time = toc(obj.timer) + obj.simulation_time_offset;
                    time_diff = simulated_time - simulation_time;
                    % avoid plotter trying catching up when simulation is
                    % slow
                    if time_diff < 0
                        obj.simulation_time_offset = obj.simulation_time_offset + time_diff;
                    else
                        pause(time_diff);
                    end

                    obj.plot(complete_plotting_info);

                    %delete field
                    obj.plotting_info_collection = rmfield(obj.plotting_info_collection, field_name);
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

    methods (Access = protected)

        function start_simulation_timer(obj)
            %START_SIMULATION_TIMER  Start the simulation timer.

            if isempty(obj.timer)
                obj.timer = tic;
            end

        end

        function plotting_info = compute_plotting_info(obj, msg)
            % compute a working plotting info object that is needed for plotting
            % input is a ros2 message
            % reconstruct matrices from list sent via ros2
            plotting_info = PlottingInfo();
            plotting_info.trajectory_predictions = reshape(msg.trajectory_predictions, 4, numel(msg.trajectory_predictions) / 4)';
            plotting_info.ref_trajectory = zeros(1, obj.scenario.options.Hp, 2);
            plotting_info.ref_trajectory(1, :, :) = reshape(msg.ref_trajectory, numel(msg.ref_trajectory) / 2, 2);
            plotting_info.priorities = msg.priorities;
            plotting_info.n_obstacles = msg.n_obstacles;
            plotting_info.n_dynamic_obstacles = msg.n_dynamic_obstacles;
            plotting_info.step = msg.step;
            plotting_info.veh_indices = msg.veh_indices;
            plotting_info.tick_now = msg.tick_now;
            plotting_info.weighted_coupling_reduced = reshape(msg.weighted_coupling_reduced, obj.scenario.options.amount, obj.scenario.options.amount)';
            plotting_info.directed_coupling = reshape(msg.directed_coupling, obj.scenario.options.amount, obj.scenario.options.amount)';
            plotting_info.belonging_vector = msg.belonging_vector;
            plotting_info.is_virtual_obstacle = reshape(msg.is_virtual_obstacle, obj.scenario.options.amount, obj.scenario.options.amount)';
        end

        function complete_plotting_info = merge_plotting_infos(obj, plotting_info_collection)
            complete_plotting_info = plotting_info_collection{1};
            complete_plotting_info.veh_indices = cellfun(@(x) x.veh_indices(1), plotting_info_collection);

            for i = 1:length(plotting_info_collection)
                info = plotting_info_collection{i};
                trajectory_predictions{i, 1} = info.trajectory_predictions;
                ref_trajectory(i, :, :) = info.ref_trajectory(1, :, :);
            end

            complete_plotting_info.trajectory_predictions = trajectory_predictions;
            complete_plotting_info.ref_trajectory = ref_trajectory;
            complete_plotting_info.priorities = cellfun(@(x) x.priorities, plotting_info_collection)';

            complete_plotting_info.belonging_vector = cellfun(@(x) x.belonging_vector, plotting_info_collection)';

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
                    complete_plotting_info.reachable_sets{i, :} = info.reachable_sets;
                end

            end

            if obj.plot_options.plot_lanelet_crossing_areas
                complete_plotting_info.lanelet_crossing_areas = cellfun(@(x) x.lanelet_crossing_areas, plotting_info_collection, 'UniformOutput', false);
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
            % Replot the currently stored plotting info with possibly changed plotting options.
            obj.plot(obj.current_plotting_info);
        end

    end

end
