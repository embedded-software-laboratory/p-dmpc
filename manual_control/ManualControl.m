classdef (Abstract) ManualControl
    
    properties (Access = protected)
        vehicle_id
        input_device_id
        joy_node
        joy_subscriber
        reader_vehicleStateList
        g29_force_feedback
    end

    methods
        function obj = ManualControl(vehicle_id, input_device_id, force_feedback_enabled)
            obj.vehicle_id = vehicle_id;
            obj.input_device_id = input_device_id;
            cmdStr = ['gnome-terminal --' ' ' 'launch_j' num2str(input_device_id) '.sh'];
            system(cmdStr);
            if force_feedback_enabled
                cmdStr = ['gnome-terminal --' ' ' 'launch_g29_force_feedback.sh'];
                system(cmdStr);
            end
        end

        function input_device_data = decode_input_data()
            % generic data from different input devices
            input_device_data = struct(); % steering und throttle
        end
    end
end