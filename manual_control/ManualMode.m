classdef ManualMode < ManualControl

    properties (Access = private)
        writer_vehicleCommandDirect
    end

    methods
        function obj = ManualMode(vehicle_id, input_device_id)
            obj = obj@ManualControl(vehicle_id, input_device_id);
            obj.joy_node = ros2node("/manual_node");
            obj.joy_subscriber = ros2subscriber(obj.joy_node, ['/j' num2str(obj.input_device_id)], "sensor_msgs/Joy");
            
            % Import IDL files from cpm library
            dds_idl_matlab = fullfile('../../cpm_lib/dds_idl_matlab/');
            assert(isfolder(dds_idl_matlab),...
                'Missing directory "%s".', dds_idl_matlab);
            assert(~isempty(dir([dds_idl_matlab, '*.m'])),...
                'No MATLAB IDL-files found in %s', dds_idl_matlab);
            addpath(dds_idl_matlab)
            
            % add QoS profile
            script_directory = fileparts([mfilename('fullpath') '.m']);
            qos_xml = fullfile(script_directory, 'qos.xml');
            assert(isfile(qos_xml),...
                'Missing QOS XML "%s"', qos_xml);
            qos_env_var = "NDDS_QOS_PROFILES";
            qos_profiles = getenv( qos_env_var );

            if ~( contains( qos_profiles, qos_xml ) )
                qos_path = ['file://' , qos_xml];
                if ~(isempty(qos_profiles))
                    qos_path = [';' qos_path];
                end
                qos_profiles = [qos_profiles, qos_path];
                
                setenv( qos_env_var , qos_profiles );
            end

            dds_domain = '21';
%             dds_domain = getenv('DDS_DOMAIN');
            obj.dds_participant = DDS.DomainParticipant( 'ManualControlLibrary::Base', str2double(dds_domain) );
            obj.writer_vehicleCommandDirect = DDS.DataWriter(DDS.Publisher(obj.dds_participant), 'VehicleCommandDirect', 'vehicleCommandDirect');
            obj.reader_vehicleState = DDS.DataReader(DDS.Subscriber(obj.dds_participant), 'VehicleState', 'vehicleState');
            obj.g29_force_feedback = G29ForceFeedback(); % TODO: use this or rewrite?

        end

        function [result, force_feedback] = MessageProcessing(obj,generic_data)
            % result should contain everything needed fot vehicleCommandDirect
            % force_feedback should contain position and torque

            vehicle_state = obj.read_vehicle_state();
            if isempty(vehicle_state)
                error('Could not read vehicle state.');
            end

            vehicle_command_direct = VehicleCommandDirect;
            vehicle_command_direct.vehicle_id = uint8(obj.vehicle_id);

            % Steering
            % Scale steering command to match the G29 + muCar combination
            theta_max_full = 3*pi;
            theta_max_g29 = 2.5*pi;
            delta_max_full = 0.98;
            delta_max = 0.4135;
            theta_delta_max = theta_max_full * delta_max / delta_max_full;
            steering = generic_data.steering * theta_max_g29 / theta_delta_max;
            
            % Limit steering angle
            steering = min(1, max( -1, steering));

            
            % Throttle
            throttle = motor_throttle_manual( ...
                generic_data.throttle, ...
                generic_data.brake, ...
                vehicle_state.speed ...
            );

            % Command
            vehicle_command_direct.motor_throttle = double(throttle);
            vehicle_command_direct.steering_servo = double(steering);
            vehicle_command_direct.header.create_stamp.nanoseconds = ... 
                uint64(generic_data.timestamp);
            vehicle_command_direct.header.valid_after_stamp.nanoseconds = ...
                uint64(generic_data.timestamp);

            result = vehicle_command_direct;

            % Force feedback
            force_feedback = obj.g29_force_feedback.compute_force_feedback_manual_mode(vehicle_state, steering);
        end

        function apply(obj, result, force_feedback)
            obj.writer_vehicleCommandDirect.write(result);
            obj.g29_force_feedback.send_message(force_feedback)
        end
    end
end