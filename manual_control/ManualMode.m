classdef ManualMode < ManualControl

    properties (Access = private)
        writer_vehicleCommandDirect
    end

    methods
        function obj = ManualMode(vehicle_id, input_device_id, force_feedback_enabled)
            obj = obj@ManualControl(vehicle_id, input_device_id, force_feedback_enabled);
            obj.joy_node = ros2node("/manual_node");
            obj.joy_subscriber = ros2node(obj.joy_node, ['/j' num2str(obj.input_device_id)], "sensor_msgs/Joy");

            % Initialize data readers/writers...
            % getenv('HOME'), 'dev/software/high_level_controller/examples/matlab' ...
            common_cpm_functions_path = fullfile( ...
            getenv('HOME'), 'dev/software/high_level_controller/examples/matlab' ...
            );
            assert(isfolder(common_cpm_functions_path), 'Missing folder "%s".', common_cpm_functions_path);
            addpath(common_cpm_functions_path);


            [~,obj.reader_vehicleStateList,~,~,~,~,~,obj.writer_vehicleCommandDirect] = init_script(1);

            if force_feedback_enabled
                obj.g29_force_feedback = G29ForceFeedback(); % TODO: use this or rewrite?
            end
        end

        function [result, force_feedback] = MessageProcessing(generic_data)
            % result should contain everything needed fot vehicleCommandDirect
            % force_feedback should contain position and torque
        end
    end
end