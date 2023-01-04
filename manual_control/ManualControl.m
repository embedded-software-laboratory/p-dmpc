classdef (Abstract) ManualControl
    
    properties (Access = private)
        vehicle_id
        input_device_id
        joy_node
        joy_subscriber
        control_mode                % 1 for Manual, 2 for Semi-Autonomous
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
    end
end