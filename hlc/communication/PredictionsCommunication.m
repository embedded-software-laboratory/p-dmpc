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
                vehicles_fallback, ...
                priority_permutation ...
            )

            arguments
                obj (1, 1) PredictionsCommunication
                time_step (1, 1) double
                predicted_areas (1, :) cell
                vehicles_fallback (:, 1) double = []
                priority_permutation (1, 1) double = 0
            end

            % vehicle send message to its topic
            obj.message_to_be_sent.time_step = int32(time_step);
            obj.message_to_be_sent.vehicle_id = int32(obj.vehicle_id);
            % which vehicles should take fallback
            obj.message_to_be_sent.vehicles_fallback = int32(vehicles_fallback);

            for i = 1:length(predicted_areas)
                obj.message_to_be_sent.predicted_areas(i).x = predicted_areas{i}(1, :)';
                obj.message_to_be_sent.predicted_areas(i).y = predicted_areas{i}(2, :)';
            end

            obj.message_to_be_sent.priority_permutation = int32(priority_permutation);

            send(obj.ros2_publisher, obj.message_to_be_sent);
        end

        function tf = is_msg_outdated( ...
                obj, ...
                message_received ...
            )

            tf = ...
                ([obj.messages_stored.vehicle_id] == ...
                message_received.vehicle_id) & ...
                ([obj.messages_stored.time_step] == ...
                message_received.time_step) & ...
                ([obj.messages_stored.priority_permutation] == ...
                message_received.priority_permutation);
        end

        function tf = is_found_message( ...
                obj, ...
                vehicle_id_subscribed, ...
                time_step, ...
                priority_permutation ...
            )

            arguments
                obj (1, 1) InterHlcCommunication
                vehicle_id_subscribed (1, 1) double
                time_step (1, 1) double
                priority_permutation (1, 1) double = 0
            end

            tf = ...
                [obj.messages_stored.vehicle_id] == ...
                int32(vehicle_id_subscribed) & ...
                [obj.messages_stored.time_step] == ...
                int32(time_step) & ...
                [obj.messages_stored.priority_permutation] == ...
                int32(priority_permutation);

        end

    end

end
