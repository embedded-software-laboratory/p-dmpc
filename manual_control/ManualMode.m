classdef ManualMode < ManualControl

    properties (Access = private)
        writer_vehicleCommandDirect
        steering_over_time (3,1) double
    end

    methods
        function obj = ManualMode(vehicle_id, input_device_id)
            obj = obj@ManualControl(vehicle_id, input_device_id);
            obj.joy_node = ros2node("/manual_node");
            obj.joy_subscriber = ros2subscriber(obj.joy_node, ['/j' num2str(obj.input_device_id)], "sensor_msgs/Joy");
            
            % Import IDL files from cpm library
            dds_idl_matlab = fullfile('../../cpm_lib/dds_idl_matlab/');
            assert(isfolder(dds_idl_matlab),...
                'Missing directory "%s".', dds_idl_matlab);
            assert(~isempty(dir([dds_idl_matlab, '*.m'])),...
                'No MATLAB IDL-files found in %s', dds_idl_matlab);
            addpath(dds_idl_matlab)
            
            dds_domain = getenv('DDS_DOMAIN');
            obj.dds_participant = DDS.DomainParticipant( 'BuiltinQosLibExp::Generic.BestEffort', str2double(dds_domain) );
            obj.writer_vehicleCommandDirect = DDS.DataWriter(DDS.Publisher(obj.dds_participant), 'VehicleCommandDirect', 'vehicleCommandDirect');
            obj.reader_vehicleState = DDS.DataReader(DDS.Subscriber(obj.dds_participant), 'VehicleState', 'vehicleState');
            obj.g29_force_feedback = G29ForceFeedback(); % TODO: use this or rewrite?

        end

        function [result, force_feedback] = MessageProcessing(obj,generic_data)
            % result should contain everything needed fot vehicleCommandDirect
            % force_feedback should contain position and torque

            vehicle_state = obj.read_vehicle_state();
            if isempty(vehicle_state)
                result = [];
                force_feedback = [];
                return
            end

            vehicle_command_direct = VehicleCommandDirect;
            vehicle_command_direct.vehicle_id = uint8(obj.vehicle_id);


            steering = generic_data.steering;
            
            throttle = motor_throttle_manual( ...
                generic_data.throttle, ...
                generic_data.brake, ...
                vehicle_state.speed ...
            );

            vehicle_command_direct.motor_throttle = double(throttle);
            vehicle_command_direct.steering_servo = double(steering);
            vehicle_command_direct.header.create_stamp.nanoseconds = ... 
                uint64(generic_data.timestamp);
            vehicle_command_direct.header.valid_after_stamp.nanoseconds = ...
                uint64(generic_data.timestamp+obj.dt_period_nanos);

            result = vehicle_command_direct;


            force_feedback = obj.compute_force_feedback_data(vehicle_state, steering);
        end


        
        function [steering_speed, steering_acceleration ] = compute_steering_derivatives(obj, steering)
            obj.steering_over_time(3) = steering;
            obj.steering_over_time = circshift(obj.steering_over_time,1);
            dt = ManualControl.dt_seconds;
            d_speed = diff(obj.steering_over_time);
            steering_speed = d_speed(1) / dt;
            dd_speed = diff(d_speed);
            steering_acceleration = dd_speed / dt^2;
        end

        function apply(obj, result, force_feedback)
            obj.writer_vehicleCommandDirect.write(result);
            obj.g29_force_feedback.send_message(force_feedback)
        end

        function result = compute_force_feedback_data(obj, vehicle_state, steering)
            arguments
                obj (1,1) ManualMode
                vehicle_state (1,1) VehicleState
                steering (1,1) double
            end
            % TODO simplify for smooth torque
            [steering_speed, steering_acceleration] = obj.compute_steering_derivatives(steering);
            inertia_steering_column = 0.01;
            b_ps = 3;
            k1 = 300;
            k2 = 5;
            scale = 1/18;
            L_f = 1.3;
            L_r = L_f;
            L = L_r + L_f;
            steering_angle_max_rad = 35 / 180 * pi;
            steering_angle_rad = steering * steering_angle_max_rad;
            side_slip_angle = L_r / L * steering_angle_rad;
            v_x = vehicle_state.speed / scale ;
            v_y = atan(side_slip_angle) * v_x;
            torque = -inertia_steering_column * steering_acceleration ...
                - b_ps * steering_speed ...
                + k1 * ( (v_y + L_f * vehicle_state.imu_yaw_rate) / v_x - steering_angle_rad ) ...
                - k2 * steering_angle_rad;
            result.torque = torque;
            result.position = 0;
        end
    end
end