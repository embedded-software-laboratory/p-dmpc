classdef SemiAutonomousMode < ManualControl
    methods
        function obj = SemiAutonomousMode(vehicle_id, input_device_id, force_feedback_enabled)
            obj = ManualControl(vehicle_id, input_device_id, force_feedback_enabled);
            obj.node = ros2node("/semi_autonomous_node");
        end
    end
end