classdef SemiAutonomousMode < ManualControl
% MANUAL_MODE       Specific Instance of manual controller (ManualControl) in semi-autonomous control mode
%                   TODO: This is not implemented yet

    methods
        function obj = SemiAutonomousMode(vehicle_id, input_device_id)
            obj = ManualControl(vehicle_id, input_device_id);
            obj.node = ros2node("/semi_autonomous_node");
            % TODO: init semi autonomous manual control
        end
    end
end