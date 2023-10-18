classdef PredictionsCommunication < InterHlcCommunication
    % communication class for predictions message

    properties

    end

    methods

        function obj = PredictionsCommunication()
            % create communication class to connect to ROS2 network
            % call superclass constructor
            obj = obj@InterHlcCommunication();
        end

        function send_message( ...
                obj, ...
                time_step, ...
                predicted_areas, ...
                vehs_fallback ...
            )

            arguments
                obj (1, 1) PredictionsCommunication
                time_step (1, 1) double
                predicted_areas (1, :) cell
                vehs_fallback (:, 1) double = []
            end

            % vehicle send message to its topic
            obj.message_to_be_sent.time_step = int32(time_step);
            obj.message_to_be_sent.vehicle_id = int32(obj.vehicle_id);
            % which vehicles should take fallback
            obj.message_to_be_sent.vehs_fallback = int32(vehs_fallback);

            for i = 1:length(predicted_areas)
                obj.message_to_be_sent.predicted_areas(i).x = predicted_areas{i}(1, :)';
                obj.message_to_be_sent.predicted_areas(i).y = predicted_areas{i}(2, :)';
            end

            send(obj.ros2_publisher, obj.message_to_be_sent);
        end

    end

end
