classdef SimLabDistributed < Plant
    % SIMLAB    Instance of experiment interface used for simulation in matlab.

    properties (Access = private)
        plotter % own plotter to visualize if no visualization_data_queue is given
        should_plot
        ros2_node
        publisher
        msg_to_be_sent
    end

    methods

        function obj = SimLabDistributed()
            obj = obj@Plant();
            obj.generate_plotting_info_msgs();
            obj.ros2_node = ros2node(['/plant_', num2str(obj.veh_ids(1))]);
            options = struct("History", "keeplast", "Depth", 40, "Reliability", "reliable", "Durability", "transientlocal");
            topice_name_publish = ['/plant_', num2str(obj.veh_ids(1)), '_plotting'];
            obj.publisher = ros2publisher(obj.ros2_node, topic_name_publish, "plotting_info/PlottingInfo", options);
            publisher = obj.publisher;
        end

        function setup(obj, scenario, veh_ids)
            setup@Plant(obj, scenario, veh_ids);
            obj.should_plot = obj.scenario.options.options_plot_online.is_active;
        end

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

                %filter plotting info for controlled vehicles before
                %sending
                plotting_info = plotting_info.filter(obj.scenario.options.amount, obj.scenario.options.options_plot_online);
                obj.compute_msg_to_be_sent(plotting_info)

                %todo to message
                send(obj.publisher, obj.msg_to_be_sent); % send rosmessage
                % pause so that `keyPressCallback()` can be executed in time
                pause(0.01);
            end

        end

        function compute_msg_to_be_sent(obj, plotting_info)
            obj.msg_to_be_sent.trajectory_predictions = reshape(plotting_info.trajectory_predictions', 1, numel(plotting_info.trajectory_predictions));
            obj.msg_to_be_sent.ref_trajectory = reshape(plotting_info.ref_trajectory', 1, numel(plotting_info.ref_trajectory));
            obj.msg_to_be_sent.n_obstacles = 0;
            obj.msg_to_be_sent.n_dynamic_obstacles = 0;
            obj.msg_to_be_sent.step = plotting_info.step;
            obj.msg_to_be_sent.vehicle_indices = plotting_info.vehicle_indices;
            obj.msg_to_be_sent.tick_now = plotting_info.tick_now;
            obj.msg_to_be_sent.weighted_coupling_reduced = reshape(plotting_info.weighted_coupling_reduced', 1, numel(plotting_info.weighted_coupling_reduced));
            obj.msg_to_be_sent.directed_coupling = reshape(plotting_info.directed_coupling', 1, numel(plotting_info.directed_coupling));
            obj.msg_to_be_sent.belonging_vector = plotting_info.belonging_vector;
            obj.msg_to_be_sent.coupling_info = reshape(plotting_info.coupling_info', 1, numel(plotting_info.coupling_info));
        end

        function generate_plotting_info_msgs(obj)
            msgList = ros2("msg", "list"); % get all ROS 2 message types

            if ((sum(cellfun(@(c)strcmp(c, 'plotting_info/PlottingInfo'), msgList)) == 0))
                [file_path, ~, ~] = fileparts(mfilename('fullpath'));
                disp('Generating ROS 2 custom message type for distributed plotting...')

                try
                    ros2genmsg([file_path, filesep, 'plotting_info']);
                catch ME
                    disp(['If all environments for ros2genmsg() are prepared but still failed, try to move the whole folder to a ' ...
                          'shallower path and run again if you use Windows machine, which sadly has a max path limit constraint.'])
                    throw(ME)
                end

            else
                disp(['No generation of ROS 2 custom message type for distributed plotting, since at least "plotting_info/PlottingInfo" ' ...
                      'message exists. If message types are missing regenerate all by removing the folder matlab_msg_gen...'])
            end

        end

        function got_stop = is_stop(obj)
            got_stop = false;

            if obj.k >= obj.scenario.options.k_end
                disp('Simulation will be stopped as the defined simulation duration is reached.')
                got_stop = true;
            end

        end

        function end_run(obj)
            disp('End')
        end

    end

end
