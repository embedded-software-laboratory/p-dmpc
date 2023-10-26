classdef DecisionCommunication < InterHlcCommunication
    % communication class for predictions message

    properties

    end

    methods

        function obj = DecisionCommunication()
            % create communication class to connect to ROS2 network
            % call superclass constructor
            obj = obj@InterHlcCommunication();
        end

        function send_message( ...
                obj, ...
                time_step, ...
                decision ...
            )

            arguments
                obj (1, 1) DecisionCommunication
                time_step (1, 1) double
                decision (:, 1) double
            end

            % vehicle send message to its topic
            obj.message_to_be_sent.time_step = int32(time_step);
            obj.message_to_be_sent.vehicle_id = int32(obj.vehicle_id);
            obj.message_to_be_sent.decision = double(decision);

            send(obj.ros2_publisher, obj.message_to_be_sent);
        end

    end

end
