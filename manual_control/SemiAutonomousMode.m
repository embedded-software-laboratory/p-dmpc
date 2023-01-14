classdef SemiAutonomousMode < ManualControl
    methods
        function obj = SemiAutonomousMode(vehicle_id, input_device_id)
            obj = ManualControl(vehicle_id, input_device_id);
            obj.node = ros2node("/semi_autonomous_node");
            % TODO: init semi autonomous manual control
        end

        function result = compute_force_feedback_data(obj,vehicle_state)
            obj.force_feedback_angle = vehicle_state.steering_servo;
            result = struct('angle',obj.force_feedback_angle,'torque',0.3);
        end
    end
end