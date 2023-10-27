classdef SolutionCostCommunication < InterHlcCommunication
    % communication class for predictions message

    properties

    end

    methods

        function obj = SolutionCostCommunication()
            % create communication class to connect to ROS2 network
            % call superclass constructor
            obj = obj@InterHlcCommunication();
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
