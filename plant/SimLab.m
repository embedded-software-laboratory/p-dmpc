classdef SimLab < Plant
    % SIMLAB    Instance of experiment interface used for simulation in matlab.

    properties (Access = private)
        plotter % own plotter to visualize if no visualization_data_queue is given
        visualization_data_queue
        use_visualization_data_queue
        should_plot
    end

    methods

        function obj = SimLab()
            obj = obj@Plant();
            obj.use_visualization_data_queue = false;
        end

        function visualization_data_queue = set_visualization_data_queue(obj)
            obj.visualization_data_queue = parallel.pool.DataQueue;
            visualization_data_queue = obj.visualization_data_queue;
            obj.use_visualization_data_queue = true;
        end

        function setup(obj, scenario, custom_vehicle_ids)
            % during dry run custom ids might be passed in,
            % otherwise use vehicle IDs corresponging to path IDs
            if exist('custom_vehicle_ids', 'var')
                scenario.set_vehicle_ids(custom_vehicle_ids);
            else
                scenario.set_vehicle_ids(scenario.options.path_ids);
            end

            setup@Plant(obj, scenario);
            obj.should_plot = obj.scenario.options.options_plot_online.is_active;

            if obj.should_plot && ~obj.use_visualization_data_queue
                warning("SimLab is using its own plotter with indices:");
                disp(obj.indices_in_vehicle_list);
                obj.plotter = PlotterOnline(obj.scenario, obj.indices_in_vehicle_list);
            end

        end

        % TODO  function still in use?
        %         function update(obj)
        %             obj.cur_node = node(0, [obj.scenario.vehicles(:).trim_config], [obj.scenario.vehicles(:).x_start]', [obj.scenario.vehicles(:).y_start]', [obj.scenario.vehicles(:).yaw_start]', zeros(obj.scenario.options.amount,1), zeros(obj.scenario.options.amount,1));
        %         end

        function [x0, trim_indices] = measure(obj)
            [x0, trim_indices] = obj.measure_node();
        end

        function apply(obj, info, result, k, ~)
            % simulate change of state
            for iVeh = obj.indices_in_vehicle_list
                obj.cur_node(iVeh, :) = info.next_node(iVeh, :);
            end

            obj.k = k;

            if obj.should_plot
                % visualize time step
                % tick_now = obj.scenario.options.tick_per_step + 2; % plot of next time step. set to 1 for plot of current time step
                tick_now = 1; % plot of next time step. set to 1 for plot of current time step
                plotting_info = PlottingInfo(obj.indices_in_vehicle_list, result, obj.k, tick_now);

                if obj.use_visualization_data_queue
                    %filter plotting info for controlled vehicles before
                    %sending
                    plotting_info = plotting_info.filter(obj.scenario.options.amount, obj.scenario.options.options_plot_online);
                    send(obj.visualization_data_queue, plotting_info);
                else
                    % wait to simulate realtime plotting
                    pause(obj.scenario.options.dt - result.step_time(obj.k))
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

            if obj.k >= obj.scenario.options.k_end
                disp('Simulation will be stopped as the defined simulation duration is reached.')
                got_stop = true;
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
