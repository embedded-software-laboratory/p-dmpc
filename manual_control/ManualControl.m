classdef (Abstract) ManualControl < handle
    
    properties (Access = protected)
        vehicle_id
        input_device_id
        joy_node
        joy_subscriber
        reader_vehicleState
        g29_force_feedback
        force_feedback_angle
        dt_period_nanos = uint64(4*1e7)
        matlab_participant
    end

    methods
        function obj = ManualControl(vehicle_id, input_device_id)
            obj.vehicle_id = vehicle_id;
            obj.input_device_id = input_device_id;
            obj.force_feedback_angle = 0;
            cmdStr = ['gnome-terminal --' ' ' './manual_control/launch_j' num2str(input_device_id) '.sh'];
            system(cmdStr);
            cmdStr = ['gnome-terminal --' ' ' './manual_control/launch_g29_force_feedback.sh'];
            system(cmdStr);
            disp('terminal launch finished')
        end

        function input_device_data = decode_input_data(obj)
            % generic data from different input devices
            % TODO separate accelerator pedal and brake pedal
            input_device_data = struct('throttle',0,'steering',0,'timestamp',uint64(0)); % throttle,steering,timestamp
            joy_msg = obj.joy_subscriber.LatestMessage;

            timestamp_sec = uint64(joy_msg.header.stamp.sec);
            timestamp_nanosec = uint64(joy_msg.header.stamp.nanosec);
            input_device_data.timestamp = uint64(timestamp_sec*1e9 + timestamp_nanosec);

            switch length(joy_msg.buttons)
                case 25 % wheel
                    input_device_data.steering = joy_msg.axes(1);
                    if joy_msg.axes(3) >= 0 %throttle pedal
                        input_device_data.throttle = joy_msg.axes(3);
                    elseif joy_msg.axes(4) >= 0 %brake pedal
                        input_device_data.throttle = (-1) * joy_msg.axes(4);
                    end
                 case 11 % gamepad - not necessary right now
                    input_device_data.steering = joy_msg.axes(1);
                    if joy_msg.axes(3) >= 0 %throttle pedal
                        input_device_data.throttle = joy_msg.axes(3);
                    elseif joy_msg.axes(6) >= 0 %brake pedal
                        input_device_data.throttle = (-1) * joy_msg.axes(4);
                    end
            end
        end

        function force_feedback_data = calculate_force_feedback_data(obj) 
            [sample, ~, sample_count, ~] = obj.reader_vehicleState.take();
            for i = 1 : sample_count
                if sample(i).vehicle_id == obj.vehicle_id
                    obj.force_feedback_angle = sample(i).steering_servo;
                end
            end
            force_feedback_data = struct('angle',obj.force_feedback_angle,'torque',0.3);
        end
    end
end