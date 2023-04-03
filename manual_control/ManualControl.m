classdef (Abstract) ManualControl < handle
    % MANUALCONTROL     Abstract interface class implementing common functions for different control modes

    properties (Access = protected)
        vehicle_id
        input_device_id
        joy_node
        joy_subscriber
        reader_vehicleState
        g29_force_feedback G29ForceFeedback = G29ForceFeedback();
        dds_participant DDS.DomainParticipant
    end

    properties (Constant)
        dt_seconds = 0.02;
    end

    methods

        function obj = ManualControl(vehicle_id, input_device_id)
            obj.vehicle_id = vehicle_id;
            obj.input_device_id = input_device_id;
            % TODO tmux
            cmdStr = ['gnome-terminal --' ' ' './manual_control/launch_j' num2str(input_device_id) '.sh'];
            system(cmdStr);
            cmdStr = ['gnome-terminal --' ' ' './manual_control/launch_g29_force_feedback.sh'];
            system(cmdStr);
            disp('terminal launch finished')
        end

        function input_device_data = decode_input_data(obj)
            % generic data from different input devices
            input_device_data = struct( ...
                'throttle', 0, ...
                'brake', 0, ...
                'steering', 0, ...
                'timestamp', uint64(0) ...
            );
            joy_msg = obj.joy_subscriber.LatestMessage;

            if isempty(joy_msg)
                error('Could not read joy input.');
            end

            timestamp_sec = uint64(joy_msg.header.stamp.sec);
            timestamp_nanosec = uint64(joy_msg.header.stamp.nanosec);
            input_device_data.timestamp = uint64(timestamp_sec * 1e9 + timestamp_nanosec);

            switch length(joy_msg.buttons)
                case 25 % wheel
                    brake_axes = joy_msg.axes(4);
                case 11 % gamepad - not necessary right now
                    brake_axes = joy_msg.axes(6);
            end

            input_device_data.steering = joy_msg.axes(1);
            input_device_data.throttle = joy_msg.axes(3);
            input_device_data.brake = brake_axes;
        end

        function result = read_vehicle_state(obj)
            result = [];
            [sample, ~, sample_count, ~] = obj.reader_vehicleState.take();

            for i = 1:sample_count

                if sample(i).vehicle_id == obj.vehicle_id
                    result = sample(i);
                end

            end

        end

    end

end
