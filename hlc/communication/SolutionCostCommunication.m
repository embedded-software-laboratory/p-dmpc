classdef SolutionCostCommunication < InterHlcCommunication
    % communication class for predictions message

    properties

    end

    methods

        function obj = SolutionCostCommunication( ...
                vehicle_id, ...
                ros2_node, ...
                topic_name, ...
                message_type ...
            )

            % create communication class to connect to ROS2 network
            arguments
                vehicle_id (1, 1) double
                ros2_node (1, 1) ros2node
                topic_name (1, :) char
                message_type (1, :) char
            end

            % create communication class to connect to ROS2 network
            % call superclass constructor
            obj = obj@InterHlcCommunication( ...
                vehicle_id, ...
                ros2_node, ...
                topic_name, ...
                message_type ...
            );
        end

        function send_message( ...
                obj, ...
                time_step, ...
                solution_cost ...
            )

            arguments
                obj (1, 1) SolutionCostCommunication
                time_step (1, 1) double
                solution_cost (:, 1) double
            end

            % vehicle send message to its topic
            obj.message_to_be_sent.time_step = int32(time_step);
            obj.message_to_be_sent.vehicle_id = int32(obj.vehicle_id);
            obj.message_to_be_sent.solution_cost = double(solution_cost);

            send(obj.ros2_publisher, obj.message_to_be_sent);
        end

    end

end
