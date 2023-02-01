classdef ManualMode < ManualControl

    properties (Access = private)
        writer_vehicleCommandDirect
    end

    properties (Access = public, Constant)
        % Related to throttle
        % Parameters BMW 320i, see https://gitlab.lrz.de/tum-cps/commonroad-vehicle-models/-/blob/master/vehicleModels_commonRoad.pdf
        scale = 1/18;
        max_acceleration = 11.5 * ManualMode.scale;
        switching_speed = 7.22 * ManualMode.scale;
        p5 = -1;
        p6 = 4.5;
        p7 = 1.34;
        p_stop = 1;
    end

    methods

        function obj = ManualMode(vehicle_id, input_device_id)
            obj = obj@ManualControl(vehicle_id, input_device_id);
            obj.joy_node = ros2node("/manual_node");
            obj.joy_subscriber = ros2subscriber(obj.joy_node, ['/j' num2str(obj.input_device_id)], "sensor_msgs/Joy");

            % Import IDL files from cpm library
            dds_idl_matlab = fullfile('../../cpm_lib/dds_idl_matlab/');
            assert(isfolder(dds_idl_matlab), ...
                'Missing directory "%s".', dds_idl_matlab);
            assert(~isempty(dir([dds_idl_matlab, '*.m'])), ...
                'No MATLAB IDL-files found in %s', dds_idl_matlab);
            addpath(dds_idl_matlab)

            % add QoS profile
            script_directory = fileparts([mfilename('fullpath') '.m']);
            qos_xml = fullfile(script_directory, 'qos.xml');
            assert(isfile(qos_xml), ...
                'Missing QOS XML "%s"', qos_xml);
            qos_env_var = "NDDS_QOS_PROFILES";
            qos_profiles = getenv(qos_env_var);

            if ~(contains(qos_profiles, qos_xml))
                qos_path = ['file://', qos_xml];

                if ~(isempty(qos_profiles))
                    qos_path = [';' qos_path];
                end

                qos_profiles = [qos_profiles, qos_path];

                setenv(qos_env_var, qos_profiles);
            end

            dds_domain = '21';
            %             dds_domain = getenv('DDS_DOMAIN');
            obj.dds_participant = DDS.DomainParticipant('ManualControlLibrary::Base', str2double(dds_domain));
            obj.writer_vehicleCommandDirect = DDS.DataWriter(DDS.Publisher(obj.dds_participant), 'VehicleCommandDirect', 'vehicleCommandDirect');
            obj.reader_vehicleState = DDS.DataReader(DDS.Subscriber(obj.dds_participant), 'VehicleState', 'vehicleState');
            obj.reader_vehicleState.WaitSet = 1;
            obj.reader_vehicleState.WaitSetTimeout = 0.03; %vehicle sends in 20 ms rate

            % Sometimes, the vehicleState messages are not received.
            disp('wait for vehicleState message')
            obj.wait_for_vehicle_state();

        end

        function [result, force_feedback] = MessageProcessing(obj, generic_data)
            % result should contain everything needed fot vehicleCommandDirect
            % force_feedback should contain position and torque

            vehicle_state = obj.read_vehicle_state();

            if isempty(vehicle_state)
                error('Could not read vehicle state.');
            end

            vehicle_command_direct = VehicleCommandDirect;
            vehicle_command_direct.vehicle_id = uint8(obj.vehicle_id);

            % Steering
            % Scale steering command to match the G29 + muCar combination
            theta_max_full = 3 * pi;
            theta_max_g29 = 2.5 * pi;
            delta_max_full = 0.98;
            delta_max = 0.4135;
            theta_delta_max = theta_max_full * delta_max / delta_max_full;
            steering = generic_data.steering * theta_max_g29 / theta_delta_max;

            % Limit steering angle
            steering = min(1, max(-1, steering));

            % Throttle
            throttle = ManualMode.motor_throttle( ...
            generic_data.throttle, ...
                generic_data.brake, ...
                vehicle_state.speed ...
            );

            % Command
            vehicle_command_direct.motor_throttle = double(throttle);
            vehicle_command_direct.steering_servo = double(steering);
            vehicle_command_direct.header.create_stamp.nanoseconds = ...
                uint64(generic_data.timestamp);
            vehicle_command_direct.header.valid_after_stamp.nanoseconds = ...
                uint64(generic_data.timestamp);

            result = vehicle_command_direct;

            % Force feedback
            force_feedback = obj.g29_force_feedback.compute_force_feedback_manual_mode(vehicle_state, steering);
        end

        function apply(obj, result, force_feedback)
            obj.writer_vehicleCommandDirect.write(result);
            obj.g29_force_feedback.send_message(force_feedback)
        end

    end
    methods (Static)
        function result = motor_throttle( ...
                accelerator_pedal_position, brake_pedal_position, speed ...
            )

            arguments
                accelerator_pedal_position (1, 1) double = -1;
                brake_pedal_position (1, 1) double = -1;
                speed (1, 1) double = 0;
            end

            acceleration_desired = ManualMode.compute_desired_acceleration( ...
                accelerator_pedal_position, brake_pedal_position, speed ...
            );

            result = ManualMode.compute_motor_throttle(acceleration_desired, speed);

        end

        function result = compute_desired_acceleration( ...
                accelerator_pedal_position, brake_pedal_position, speed ...
            )
            acceleration_from_brake = ManualMode.motor_throttle_from_pedal_and_max_acceleration( ...
                brake_pedal_position, ManualMode.compute_min_acceleration() ...
            );

            acceleration_from_accelerator = ManualMode.motor_throttle_from_pedal_and_max_acceleration( ...
                accelerator_pedal_position, ManualMode.compute_max_acceleration(speed) ...
            );
            result = acceleration_from_brake + acceleration_from_accelerator;
        end

        function result = compute_min_acceleration()
            result = -ManualMode.max_acceleration;
        end

        function result = compute_max_acceleration(speed)

            is_wheel_slip = (speed < ManualMode.switching_speed);
            result = zeros(size(speed));
            result(is_wheel_slip) = ManualMode.max_acceleration;
            result(~is_wheel_slip) = ManualMode.max_acceleration * ManualMode.switching_speed ./ speed(~is_wheel_slip);

        end

        function result = compute_motor_throttle(acceleration_desired, speed)
            % https://www.sciencedirect.com/science/article/pii/S2405896320324319

            is_braking = (acceleration_desired < 0);
            is_stop = (speed < 0.1);

            if (is_braking && is_stop)
                result = -ManualMode.p_stop * speed;
            else
                x = (acceleration_desired - ManualMode.p5 * speed) / ManualMode.p6;
                result = sign(x) * nthroot(abs(x), ManualMode.p7);
            end
            % TODO Different dynamics for braking
            result = min(1, max(-1, result));

        end


        function result = motor_throttle_from_pedal_and_max_acceleration( ...
                pedal_position, max_acceleration ...
            )
            min_acceleration = 0;
            d_acceleration = max_acceleration - min_acceleration;
            min_pedal_position = -1;
            max_pedal_position = 1;
            d_pedal = max_pedal_position - min_pedal_position;
            result = (pedal_position - min_pedal_position) / d_pedal * d_acceleration ...
                + min_acceleration;
        end

    end

    methods (Access = private)
        function wait_for_vehicle_state(obj)
            % TODO Remove
            wait_set_timout = obj.reader_vehicleState.WaitSetTimeout;
            obj.reader_vehicleState.WaitSetTimeout = 60;
            vehicle_state = obj.read_vehicle_state();
            if isempty(vehicle_state)
                error("could not read vehicle state")
            end
            obj.reader_vehicleState.WaitSetTimeout = wait_set_timout;
        end
    end

end
