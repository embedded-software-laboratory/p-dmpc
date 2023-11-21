classdef SimLabDistributed < Plant
    % SIMLAB    Instance of experiment interface used for simulation in matlab.

    properties (Access = private)
        should_plot
        ros2_node
        publisher
        msg_to_be_sent
    end

    methods

        function obj = SimLabDistributed()
            disp('using distributed simulation')
            obj = obj@Plant();
        end

        function setup(obj, options, all_vehicle_ids, controlled_vehicle_ids)

            arguments
                obj (1, 1) SimLabDistributed
                options (1, 1) Config
                all_vehicle_ids (1, :) uint8
                controlled_vehicle_ids (1, 1) uint8 = all_vehicle_ids(1)
            end

            setup@Plant(obj, options, all_vehicle_ids, controlled_vehicle_ids);

            % create scenario adapter to get scenario
            % with SimLabDistributed only BuiltScenario can be used
            scenario_adapter = BuiltScenario();
            scenario_adapter.init(options, obj);

            % all vehicles have the same initial speed and steering
            initial_speed = 0;
            initial_steering = 0;

            % set initial vehicle measurements
            for i_vehicle = 1:options.amount
                obj.measurements(i_vehicle) = PlantMeasurement( ...
                    scenario_adapter.scenario.vehicles(i_vehicle).x_start, ...
                    scenario_adapter.scenario.vehicles(i_vehicle).y_start, ...
                    scenario_adapter.scenario.vehicles(i_vehicle).yaw_start, ...
                    initial_speed, ...
                    initial_steering ...
                );
            end

            obj.should_plot = obj.options_plot_online.is_active;
            obj.generate_plotting_info_msgs();
            obj.ros2_node = ros2node(['/plant_', num2str(obj.controlled_vehicle_ids(1))]);
            qos_options = struct("History", "keeplast", "Depth", 40, "Reliability", "reliable", "Durability", "transientlocal");
            topic_name_publish = ['/plant_plotting'];
            obj.publisher = ros2publisher(obj.ros2_node, topic_name_publish, "plotting_info/PlottingInfo", qos_options);
            obj.msg_to_be_sent = ros2message("plotting_info/PlottingInfo");
        end

        function [cav_measurements, hdv_measurements] = measure(obj)
            cav_measurements = obj.measurements;
            hdv_measurements = [];
        end

        function apply(obj, info, experiment_result, k, mpa)
            % simulate change of state
            for iVeh = obj.indices_in_vehicle_list
                obj.measurements(iVeh) = PlantMeasurement( ...
                    info.next_node(iVeh, NodeInfo.x), ...
                    info.next_node(iVeh, NodeInfo.y), ...
                    info.next_node(iVeh, NodeInfo.yaw), ...
                    mpa.trims(info.next_node(iVeh, NodeInfo.trim)).speed, ...
                    mpa.trims(info.next_node(iVeh, NodeInfo.trim)).steering ...
                );
            end

            if obj.should_plot
                % visualize time step
                tick_now = 1; % plot of next time step. set to 1 for plot of current time step
                plotting_info = PlottingInfo(obj.indices_in_vehicle_list, experiment_result, k, tick_now);

                %filter plotting info for controlled vehicles before
                %sending
                plotting_info = plotting_info.filter(obj.amount, obj.options_plot_online);
                obj.compute_msg_to_be_sent(plotting_info)

                send(obj.publisher, obj.msg_to_be_sent); % send rosmessage
            end

        end

        function compute_msg_to_be_sent(obj, plotting_info)
            % fill msg to be sent to plotting device with plotting_info
            % all matrices must be converted into list to comply with ros 2 message format
            obj.msg_to_be_sent.trajectory_predictions = reshape(plotting_info.trajectory_predictions', 1, numel(plotting_info.trajectory_predictions));
            obj.msg_to_be_sent.ref_trajectory = reshape(squeeze(plotting_info.ref_trajectory), 1, numel(plotting_info.ref_trajectory));
            obj.msg_to_be_sent.priorities = int32(plotting_info.priorities);
            obj.msg_to_be_sent.n_obstacles = int32(0);
            obj.msg_to_be_sent.n_dynamic_obstacles = int32(0);
            obj.msg_to_be_sent.step = int32(plotting_info.step);
            obj.msg_to_be_sent.veh_indices = int32(plotting_info.veh_indices);
            obj.msg_to_be_sent.tick_now = int32(plotting_info.tick_now);
            obj.msg_to_be_sent.weighted_coupling_reduced = reshape(plotting_info.weighted_coupling_reduced', 1, numel(plotting_info.weighted_coupling_reduced));
            obj.msg_to_be_sent.directed_coupling = int32(reshape(plotting_info.directed_coupling', 1, numel(plotting_info.directed_coupling)));
            obj.msg_to_be_sent.belonging_vector = int32(plotting_info.belonging_vector);
            obj.msg_to_be_sent.is_virtual_obstacle = int32(reshape(plotting_info.is_virtual_obstacle', 1, numel(plotting_info.is_virtual_obstacle)));
        end

        function generate_plotting_info_msgs(obj)
            % generate message needed for plotting via ros
            msgList = ros2("msg", "list"); % get all ROS 2 message types
            % check if message type is present
            if ((sum(cellfun(@(c)strcmp(c, 'plotting_info/PlottingInfo'), msgList)) == 0))
                [file_path, ~, ~] = fileparts(mfilename('fullpath'));
                disp('Generating ROS 2 custom message type for distributed plotting...')

                try
                    ros2genmsg(file_path);
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

        function got_stop = is_stop(~)
            got_stop = false;
        end

        function end_run(obj)
            disp('End')
            obj.msg_to_be_sent.step = int32(-1);
            send(obj.publisher, obj.msg_to_be_sent); % send end message
        end

    end

end
