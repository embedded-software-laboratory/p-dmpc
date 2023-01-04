classdef ManualMode < ManualControl
    methods
        function obj = ManualMode(vehicle_id, input_device_id, force_feedback_enabled)
            obj = ManualControl(vehicle_id, input_device_id, force_feedback_enabled);
            obj.node = ros2node("/manual_node");
        end
    end
end