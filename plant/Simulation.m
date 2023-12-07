classdef Simulation < Plant
    % Simulation    Instance of experiment interface used for simulation in matlab.

    properties (Access = private)
        should_plot
        ros2_node
        publisher
        msg_to_be_sent
        subscriber
        k = 0;

        should_sync
        highest_sync_step = 0;
    end

    methods

        function obj = Simulation()
            disp('using distributed simulation')
            obj = obj@Plant();
        end

        function setup(obj, options, all_vehicle_ids, controlled_vehicle_ids)

            arguments
                obj (1, 1) Simulation
                options (1, 1) Config
                all_vehicle_ids (1, :) uint8
                controlled_vehicle_ids (1, :) uint8 = all_vehicle_ids
            end

            setup@Plant(obj, options, all_vehicle_ids, controlled_vehicle_ids);

            % create scenario adapter to get scenario
            % with Simulation only BuiltScenario can be used
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
            obj.should_sync = ~options.compute_in_parallel && obj.should_plot;
            Simulation.generate_plotting_info_msgs();
            % TODO Object creation in HLC factory. One ros2node
            obj.ros2_node = ros2node(['/plant_', num2str(obj.controlled_vehicle_ids(1))]);

            qos_config = struct( ...
                History = "keeplast", ...
                Depth = 40, ...
                Reliability = "reliable", ...
                Durability = "volatile" ...
            );
            % TODO visualization in separate class
            topic_name_publish = '/plotting';
            obj.publisher = ros2publisher(obj.ros2_node, topic_name_publish, "plotting_info/PlottingInfo", qos_config);
            obj.msg_to_be_sent = ros2message("plotting_info/PlottingInfo");

            obj.subscriber = ros2subscriber( ...
                obj.ros2_node, ...
                '/plant_sync', ...
                'std_msgs/Int32', ...
                @obj.sync_callback, ...
                qos_config ...
            );
        end

        function [cav_measurements, hdv_measurements] = measure(obj)

            if obj.should_sync
                % wait for all vehicles to finish their computation
                while obj.highest_sync_step < obj.k
                    pause(10 * 1e-3)
                end

            end

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

                if obj.should_plot
                    % visualize time step
                    tick_now = 1; % plot of next time step. set to 1 for plot of current time step
                    plotting_info = PlottingInfo(iVeh, experiment_result, k, tick_now);

                    %filter plotting info for controlled vehicles before
                    %sending
                    plotting_info = plotting_info.filter(obj.amount, obj.options_plot_online);
                    obj.compute_msg_to_be_sent(plotting_info)

                    send(obj.publisher, obj.msg_to_be_sent); % send rosmessage
                end

            end

            obj.k = k;

        end

        function compute_msg_to_be_sent(obj, plotting_info)
            % fill msg to be sent to plotting device with plotting_info
            % all matrices must be converted into list to comply with ros 2 message format
            obj.msg_to_be_sent.trajectory_predictions = reshape(plotting_info.trajectory_predictions', 1, []);
            obj.msg_to_be_sent.ref_trajectory = reshape(squeeze(plotting_info.ref_trajectory), 1, []);
            obj.msg_to_be_sent.n_obstacles = int32(0);
            obj.msg_to_be_sent.n_dynamic_obstacles = int32(0);
            obj.msg_to_be_sent.step = int32(plotting_info.step);
            obj.msg_to_be_sent.veh_indices = int32(plotting_info.veh_indices);
            obj.msg_to_be_sent.tick_now = int32(plotting_info.tick_now);
            obj.msg_to_be_sent.weighted_coupling_reduced = reshape(plotting_info.weighted_coupling_reduced', 1, []);
            obj.msg_to_be_sent.directed_coupling = uint8(reshape(plotting_info.directed_coupling', 1, []));
            obj.msg_to_be_sent.directed_coupling_sequential = uint8(reshape(full(plotting_info.directed_coupling_sequential'), 1, []));
            obj.msg_to_be_sent.is_virtual_obstacle = int32(reshape(plotting_info.is_virtual_obstacle', 1, []));
        end

        function got_stop = is_stop(~)
            got_stop = false;
        end

        function end_run(obj)
            disp('End')
            obj.msg_to_be_sent.step = int32(-1);
            send(obj.publisher, obj.msg_to_be_sent); % send end message
        end

        function sync_callback(obj, msg)
            obj.highest_sync_step = double(max(obj.highest_sync_step, double(msg.data)));
        end

    end

    methods (Static)

        function generate_plotting_info_msgs()
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

    end

end
