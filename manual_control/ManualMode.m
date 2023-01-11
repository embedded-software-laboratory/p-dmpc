classdef ManualMode < ManualControl

    properties (Access = private)
        writer_vehicleCommandDirect
        timer_send
    end

    methods
        function obj = ManualMode(vehicle_id, input_device_id)
            obj = obj@ManualControl(vehicle_id, input_device_id);
            obj.joy_node = ros2node("/manual_node");
            obj.joy_subscriber = ros2subscriber(obj.joy_node, ['/j' num2str(obj.input_device_id)], "sensor_msgs/Joy");

            % Initialize data readers/writers...
            common_cpm_functions_path = fullfile( ...
            getenv('HOME'), 'dev/software/high_level_controller/examples/matlab' ...
            );
            assert(isfolder(common_cpm_functions_path), 'Missing folder "%s".', common_cpm_functions_path);
            addpath(common_cpm_functions_path);
            
            
            % Import IDL files from cpm library
            dds_idl_matlab = fullfile('../../cpm_lib/dds_idl_matlab/');
            assert(isfolder(dds_idl_matlab),...
                'Missing directory "%s".', dds_idl_matlab);
            assert(~isempty(dir([dds_idl_matlab, '*.m'])),...
                'No MATLAB IDL-files found in %s', dds_idl_matlab);
            addpath(dds_idl_matlab)

            obj.matlab_participant = DDS.DomainParticipant('BuiltinQosLibExp::Generic.BestEffort',21);
            obj.writer_vehicleCommandDirect = DDS.DataWriter(DDS.Publisher(obj.matlab_participant), 'VehicleCommandDirect', 'vehicleCommandDirect');
            obj.reader_vehicleState = DDS.DataReader(DDS.Subscriber(obj.matlab_participant), 'VehicleState', 'vehicleState');
            obj.g29_force_feedback = G29ForceFeedback(); % TODO: use this or rewrite?

            obj.timer_send = tic;
        end

        function [result, force_feedback] = MessageProcessing(obj,generic_data)
            % result should contain everything needed fot vehicleCommandDirect
            % force_feedback should contain position and torque
            vehicle_command_direct = VehicleCommandDirect;
            vehicle_command_direct.vehicle_id = uint8(obj.vehicle_id);

            throttle = 0.5 * generic_data.throttle;
            steering = 1.2 * generic_data.steering;
            
            if throttle > 0.5
                throttle = 0.5;
            elseif throttle < -0.5
                throttle = -0.5;
            end

            vehicle_command_direct.motor_throttle = double(throttle);
            vehicle_command_direct.steering_servo = double(steering);
            vehicle_command_direct.header.create_stamp.nanoseconds = ... 
                uint64(generic_data.timestamp);
            vehicle_command_direct.header.valid_after_stamp.nanoseconds = ...
                uint64(generic_data.timestamp+obj.dt_period_nanos);

            result = vehicle_command_direct;

            force_feedback = obj.calculate_force_feedback_data();
        end

        function apply(obj, result, force_feedback)
            if toc(obj.timer_send) > 0.01
                obj.timer_send = tic;
                obj.writer_vehicleCommandDirect.write(result);
            end
            obj.g29_force_feedback.send_message(force_feedback)
        end
    end
end