classdef Simulation < Plant
    % Simulation    Instance of experiment interface used for simulation in matlab.

    properties (Access = protected)

        options_plot_online
    end

    properties (Access = private)
        should_plot
        ros2_node
        publisher
        msg_to_be_sent
        k = 0;

        highest_sync_step = 0;
    end

    methods

        function obj = Simulation()
            obj = obj@Plant();
        end

        function set_ros2_node(obj, ros2_node)

            arguments
                obj (1, 1) Simulation
                ros2_node (1, 1) ros2node
            end

            obj.ros2_node = ros2_node;
        end

        function setup(obj, options, vehicle_indices_controlled)

            arguments
                obj (1, 1) Simulation
                options (1, 1) Config
                vehicle_indices_controlled (1, :) uint8
            end

            setup@Plant(obj, options);
            obj.vehicle_indices_controlled = vehicle_indices_controlled;

            % create scenario adapter to get scenario
            % with Simulation only BuiltScenario can be used
            scenario_adapter = BuiltScenario();
            scenario_adapter.init(options);

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

            obj.options_plot_online = options.options_plot_online;
            obj.should_plot = obj.options_plot_online.is_active;

            qos_config = struct( ...
                History = "keeplast", ...
                Depth = 40, ...
                Reliability = "reliable", ...
                Durability = "transientlocal" ...
            );

            topic_name_publish = '/plotting';
            obj.publisher = ros2publisher(obj.ros2_node, topic_name_publish, "veh_msgs/PlottingInfo", qos_config);
            obj.msg_to_be_sent = ros2message("veh_msgs/PlottingInfo");
        end

        function [cav_measurements, hdv_measurements] = measure(obj)
            cav_measurements = obj.measurements;
            hdv_measurements = [];
        end

        function apply(obj, info, experiment_result, k, mpa)

            % simulate change of state
            for vehicle_index = obj.vehicle_indices_controlled
                % simulate change of state
                i_vehicle = find(obj.vehicle_indices_controlled == vehicle_index);

                obj.measurements(vehicle_index) = PlantMeasurement( ...
                    info.y_predicted(1, 1, i_vehicle), ... % x
                    info.y_predicted(2, 1, i_vehicle), ... % y
                    info.y_predicted(3, 1, i_vehicle), ... % yaw
                    mpa.trims(info.predicted_trims(i_vehicle, 1)).speed, ...
                    mpa.trims(info.predicted_trims(i_vehicle, 1)).steering ...
                );

                if obj.should_plot
                    % visualize time step
                    obj.compute_msg_to_be_sent( ...
                        vehicle_index, ...
                        experiment_result.iteration_data(k), ...
                        experiment_result.control_results_info(k).y_predicted(:, :, i_vehicle), ...
                        k ...
                    )

                    send(obj.publisher, obj.msg_to_be_sent); % send rosmessage
                end

            end

            obj.k = k;

        end

        function compute_msg_to_be_sent( ...
                obj, ...
                vehicle_index, ...
                iteration_data, ...
                trajectory_prediction, ...
                k ...
            )
            % fill msg to be sent to plotting device with plotting_info
            % all matrices must be converted into list to comply with ros 2 message format
            obj.msg_to_be_sent.state = iteration_data.x0(vehicle_index, 1:3);
            obj.msg_to_be_sent.trajectory_predictions = reshape( ...
                trajectory_prediction', ...
                1, ...
                [] ...
            );
            obj.msg_to_be_sent.ref_trajectory = reshape( ...
                squeeze(iteration_data.reference_trajectory_points(vehicle_index, :, :)), ...
                1, ...
                [] ...
            );
            obj.msg_to_be_sent.n_obstacles = int32(0);
            obj.msg_to_be_sent.step = int32(k);
            obj.msg_to_be_sent.vehicle_indices = int32(vehicle_index);
            obj.msg_to_be_sent.time_seconds = k * obj.dt_seconds;
            obj.msg_to_be_sent.directed_coupling = uint8(reshape( ...
                iteration_data.directed_coupling', ...
                1, ...
                [] ...
            ));
            obj.msg_to_be_sent.directed_coupling_sequential = uint8(reshape( ...
                full(iteration_data.directed_coupling_sequential'), ...
                1, ...
                [] ...
            ));
            obj.msg_to_be_sent.weighted_coupling = reshape( ...
                iteration_data.weighted_coupling', ...
                1, ...
                [] ...
            );
        end

        function result = should_stop(~)
            result = false;
        end

        function end_run(obj)
            disp('End.')

            for i_vehicle = obj.vehicle_indices_controlled
                obj.msg_to_be_sent.step = int32(-1);
                obj.msg_to_be_sent.vehicle_indices = int32(i_vehicle);
                send(obj.publisher, obj.msg_to_be_sent);
            end

        end

        function sync_callback(obj, msg)
            obj.highest_sync_step = double(max(obj.highest_sync_step, double(msg.data)));
        end

    end

end
