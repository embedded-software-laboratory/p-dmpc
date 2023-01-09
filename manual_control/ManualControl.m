classdef (Abstract) ManualControl
    
    properties (Access = protected)
        vehicle_id
        input_device_id
        joy_node
        joy_subscriber
        reader_vehicleStateList
        g29_force_feedback
        dt_period_nanos = uint64(4*1e7)
        force_feedback_enabled
    end

    methods
        function obj = ManualControl(vehicle_id, input_device_id, force_feedback_enabled)
            obj.vehicle_id = vehicle_id;
            obj.input_device_id = input_device_id;
            obj.force_feedback_enabled = force_feedback_enabled;
            cmdStr = ['gnome-terminal --' ' ' './launch_j' num2str(input_device_id) '.sh'];
            system(cmdStr);
            if obj.force_feedback_enabled
                cmdStr = ['gnome-terminal --' ' ' './launch_g29_force_feedback.sh'];
            end
            system(cmdStr);
            disp('system... ready')
        end

        function input_device_data = decode_input_data(obj)
            % generic data from different input devices
            input_device_data = struct('throttle',0,'steering',0,'timestamp',uint64(0)); % throttle,steering,timestamp
            joy_msg = obj.joy_subscriber.LatestMessage;

            timestamp_sec = uint64(joy_msg.header.stamp.sec);
            timestamp_nanosec = uint64(joy_msg.header.stamp.nanosec);
            input_device_data.timestamp = uint64(timestamp_sec*1e9 + timestamp_nanosec);

            switch length(joy_msg.buttons)
                case 11 % gamepad
                    input_device_data.steering = joy_msg.axes(1);
                    if joy_msg.axes(3) >= 0 %throttle pedal
                        input_device_data.throttle = joy_msg.axes(3);
                    elseif joy_msg.axes(6) >= 0 %brake pedal
                        input_device_data.throttle = (-1) * joy_msg.axes(4);
                    end
                case 25 % wheel
                    input_device_data.steering = joy_msg.axes(1);
                    if joy_msg.axes(3) >= 0 %throttle pedal
                        input_device_data.throttle = joy_msg.axes(3);
                    elseif joy_msg.axes(4) >= 0 %brake pedal
                        input_device_data.throttle = (-1) * joy_msg.axes(4);
                    end
            end
        end

        function force_feedback_data = calculate_force_feedback_data(obj)
            force_feedback_data = struct('angle',0,'torque',0.3);
            [sample, ~, sample_count, ~] = obj.reader_vehicleStateList.take();
            if (sample_count ~= 1)
                warning('Received %d samples, expected 1. Correct middleware period? Missed deadline?', sample_count);
            end
            if sample_count > 0
                for i = 1:length(sample(end).state_list)
                    if sample(end).state_list(i) == obj.vehicle_id
                        force_feedback_data.angle = sample(end).state_list(iVeh).steering_servo;
                    end
                end
            end
        end
    end
end