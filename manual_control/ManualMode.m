classdef ManualMode < ManualControl

    properties (Access = private)
        writer_vehicleCommandDirect
        timer_send
    end

    methods
        function obj = ManualMode(vehicle_id, input_device_id, force_feedback_enabled)
            obj = obj@ManualControl(vehicle_id, input_device_id, force_feedback_enabled);
            obj.joy_node = ros2node("/manual_node");
            obj.joy_subscriber = ros2subscriber(obj.joy_node, ['/j' num2str(obj.input_device_id)], "sensor_msgs/Joy");

            % Initialize data readers/writers...
            common_cpm_functions_path = fullfile( ...
            getenv('HOME'), 'dev/software/high_level_controller/examples/matlab' ...
            );
            assert(isfolder(common_cpm_functions_path), 'Missing folder "%s".', common_cpm_functions_path);
            addpath(common_cpm_functions_path);

            

            [~,obj.reader_vehicleStateList,~,~,~,~,~,obj.writer_vehicleCommandDirect] = init_script(1);

            if force_feedback_enabled
                obj.g29_force_feedback = G29ForceFeedback(); % TODO: use this or rewrite?
            end

            obj.timer_send = tic;
        end

        function [result, force_feedback] = MessageProcessing(obj,generic_data)
            % result should contain everything needed fot vehicleCommandDirect
            % force_feedback should contain position and torque
            vehicle_command_direct = VehicleCommandDirect;
            vehicle_command_direct.vehicle_id = uint8(obj.vehicle_id);

            throttle = generic_data.throttle;
            steering = 1.2 * generic_data.steering;
            
            if throttle > 0.5
                throttle = 0.5;
            elseif throttle < -0.5
                throttle = -0.5;
            end

            vehicle_command_direct.motor_throttle = double(throttle);
            vehicle_command_direct.steering_servo = double(steering);
            vehicle_command_direct.header.create_stamp.nanoseconds = ... 
                uint64(generic_data.timestamp);
            vehicle_command_direct.header.valid_after_stamp.nanoseconds = ...
                uint64(generic_data.timestamp+obj.dt_period_nanos);

            result = vehicle_command_direct;

            force_feedback = obj.calculate_force_feedback_data();
        end

        function apply(obj, result, force_feedback)
            if toc(obj.timer_send) > 0.01
                obj.timer_send = tic;
                obj.writer_vehicleCommandDirect.write(result);
            end

            if obj.force_feedback_enabled
                obj.g29_force_feedback.send_message(force_feedback)
            end
        end
    end
end