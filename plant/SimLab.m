classdef SimLab < Plant
    % SIMLAB    Instance of experiment interface used for simulation in matlab.

    properties (Access = private)
        plotter % own plotter to visualize if no visualization_data_queue is given
        visualization_data_queue
        use_visualization_data_queue
        should_plot

        step_timer (1, 1) uint64 % used for tic/toc to measure the time between measure() and apply() to make realtime plotting possible
    end

    methods

        function obj = SimLab()
            obj = obj@Plant();
            obj.use_visualization_data_queue = false;
        end

        function visualization_data_queue = get_visualization_data_queue(obj)
            visualization_data_queue = obj.visualization_data_queue;
        end

        function setup(obj, options, scenario, all_vehicle_ids, controlled_vehicle_ids)

            arguments
                obj (1, 1) SimLab
                options (1, 1) Config
                scenario (1, 1) Scenario
                all_vehicle_ids (1, :) uint8 = options.path_ids
                controlled_vehicle_ids (1, :) uint8 = []
            end

            % if [] is passed in, matlab does not choose the default
            if isempty(all_vehicle_ids)
                all_vehicle_ids = options.path_ids;
            end

            % only set controlled ids to all ids after all ids have been set
            if isempty(controlled_vehicle_ids)
                controlled_vehicle_ids = all_vehicle_ids;
            end

            % check whether visualization data queue is needed and initialize if necessary
            if (options.is_prioritized ...
                    && options.compute_in_parallel ...
                    && options.options_plot_online.is_active ...
                    && isempty(obj.visualization_data_queue) ...
                )
                obj.visualization_data_queue = parallel.pool.DataQueue;
                obj.use_visualization_data_queue = true;
            end

            setup@Plant(obj, options, scenario, all_vehicle_ids, controlled_vehicle_ids);
            obj.should_plot = obj.options_plot_online.is_active;

            if obj.should_plot && ~obj.use_visualization_data_queue
                obj.plotter = PlotterOnline(options, scenario, obj.indices_in_vehicle_list);
            end

        end

        function [x0, trim_indices] = measure(obj, mpa)
            obj.step_timer = tic(); % Keep time to enable realtime plotting in apply
            [x0, trim_indices] = obj.measure_node(mpa);
        end

        function apply(obj, info, result, k, ~)
            % simulate change of state
            for iVeh = obj.indices_in_vehicle_list
                obj.cur_node(iVeh, :) = info.next_node(iVeh, :);
            end

            if obj.should_plot
                % visualize time step
                tick_now = 1; % plot of next time step. set to 1 for plot of current time step
                plotting_info = PlottingInfo(obj.indices_in_vehicle_list, result, k, tick_now);

                if obj.use_visualization_data_queue
                    %filter plotting info for controlled vehicles before
                    %sending
                    plotting_info = plotting_info.filter(obj.amount, obj.options_plot_online);
                    send(obj.visualization_data_queue, plotting_info);
                else
                    % wait to simulate realtime plotting
                    step_time = toc(obj.step_timer);
                    pause(obj.dt_seconds - step_time);
                    obj.plotter.plot(plotting_info);
                end

                % pause so that `keyPressCallback()` can be executed in time
                pause(0.01);
            end

        end

        function got_stop = is_stop(obj)
            got_stop = false;
            % idle while paused, and check if we should stop early
            if obj.should_plot && ~obj.use_visualization_data_queue

                while obj.plotter.paused

                    if obj.plotter.abort
                        disp('Aborted.');
                        got_stop = true;
                        break;
                    end

                    pause(0.1);
                end

                if obj.plotter.abort
                    disp('Aborted.');
                    got_stop = true;
                end

            end

        end

        function end_run(obj)
            disp('End')

            if obj.should_plot && ~obj.use_visualization_data_queue
                obj.plotter.close_figure();
            end

        end

    end

end
