classdef (Abstract) InterHlcCommunication < handle
    % abstract class for communication of a message

    properties
        ros2_publisher (1, 1) % ros2 publisher
        ros2_subscriber (1, 1) % ros2 subscriber
        vehicle_index (1, 1) double % vehicle index
        message_to_be_sent (1, 1) struct % initialize message type
        messages_stored (1, :) struct % list with message structs
    end

    methods (Abstract)
        send_message(obj)
    end

    methods

        function obj = InterHlcCommunication( ...
                vehicle_index, ...
                ros2_node, ...
                topic_name, ...
                message_type ...
            )

            arguments
                vehicle_index (1, 1) double
                ros2_node (1, 1) ros2node
                topic_name (1, :) char
                message_type (1, :) char
            end

            % store vehicle id to associate a message
            % with the controlled vehicle
            obj.vehicle_index = vehicle_index;
            % create ros2 message structure
            % (faster than creating every time step)
            obj.message_to_be_sent = ros2message(message_type);
            qos_config = struct( ...
                "History", "keeplast", ...
                "Depth", 40, ...
                "Durability", "transientlocal" ...
            );

            % create publisher and subscriber
            obj.create_publisher( ...
                ros2_node, ...
                topic_name, ...
                message_type, ...
                qos_config ...
            );

            obj.create_subscriber( ...
                ros2_node, ...
                topic_name, ...
                message_type, ...
                qos_config ...
            );

        end

        function create_publisher( ...
                obj, ...
                ros2_node, ...
                topic_name, ...
                message_type, ...
                qos_config ...
            )
            % all vehicles share the same topic
            % (it is accepted that a vehicle is triggered by its own message)
            obj.ros2_publisher = ros2publisher( ...
                ros2_node, ...
                topic_name, ...
                message_type, ...
                qos_config ...
            );
        end

        function create_subscriber( ...
                obj, ...
                ros2_node, ...
                topic_name, ...
                message_type, ...
                qos_config ...
            )
            % all vehicles share the same topic
            % (it is accepted that a vehicle is triggered by its own message)
            obj.ros2_subscriber = ros2subscriber( ...
                ros2_node, ...
                topic_name, ...
                message_type, ...
                @obj.callback_subscriber, ...
                qos_config ...
            );
        end

        function callback_subscriber(obj, message_received)
            % callback function that is triggered when a message is published
            % to the topic that is subscribed with this callback

            % the messages are stored in a queue as property of this class
            % the queue contains exactly one message
            % for one vehicle at one time step
            % if another message for a certain vehicle and time step is
            % received the old previous one is deleted

            % if the callback is triggered by the own message, it does not add
            % the message to the queue

            % the queue contains only messages
            % that are not older than two time steps
            % this limits the queue length to 2 * (n_vehicle-1)

            if int32(obj.vehicle_index) == message_received.vehicle_index
                % if triggered by own message, do nothing
                return
            end

            if isempty(obj.messages_stored)
                % if empty (no messages received so far)
                obj.messages_stored = message_received;
            else
                % else check if message with same vehicle id and time step
                % exists and delete this message then
                is_msg_outdated = obj.is_msg_outdated(message_received);
                obj.messages_stored(is_msg_outdated) = [];

                % append message to list
                obj.messages_stored(end + 1) = message_received;
            end

            % delete messages older than a certain time steps compared to
            % the time step of newly received message
            message_age_maximum = 2;
            is_msg_expired = ...
                [obj.messages_stored.time_step] <= ...
                (message_received.time_step - message_age_maximum);
            obj.messages_stored(is_msg_expired) = [];
        end

        function latest_msg = read_message( ...
                obj, ...
                vehicle_index_subscribed, ...
                time_step, ...
                optional ...
            )
            % read message from the given time step

            arguments
                obj (1, 1) InterHlcCommunication
                vehicle_index_subscribed (1, 1) double
                time_step (1, 1) double
                optional.throw_error (1, 1) logical = true
                optional.timeout (1, 1) double = 100.0
                optional.priority_permutation (1, 1) double = 0
            end

            % start timer for detecting timeout
            read_start = tic;

            % initialize read_time
            read_time = 0;

            % initialize returned message
            % used if timeout without error
            latest_msg = struct([]);

            while (read_time <= optional.timeout)
                % remark to pause before continue:
                % pause is necessary that MATLAB can executed the callback
                % function when the while loop is running

                % measure time to detect timeout
                read_time = toc(read_start);

                % jump to next loop iteration when message queue is empty
                if isempty(obj.messages_stored)
                    pause(1e-4)
                    continue
                end

                % find messages by vehicle id and time step
                is_found_message = obj.is_found_message(vehicle_index_subscribed, time_step, optional.priority_permutation);

                % jump to next loop iteration if no message is found
                if ~any(is_found_message)
                    pause(1e-4)
                    continue
                end

                % only one message should be found
                latest_msg = obj.messages_stored(is_found_message);
                return
            end

            if optional.throw_error
                error(['Unable to receive the current message ', ...
                       'of step %d from vehicle %d within %d seconds'], ...
                    time_step, vehicle_index_subscribed, optional.timeout)
            end

        end

        function latest_msg = read_latest_message( ...
                obj, ...
                vehicle_index_subscribed ...
            )
            % read latest available message from the given subscriber

            arguments
                obj (1, 1) InterHlcCommunication
                vehicle_index_subscribed (1, 1) double
            end

            % initialize returned message
            % used if queue is empty or no message is found
            latest_msg = struct([]);

            if isempty(obj.messages_stored)
                return
            end

            is_found_message = ...
                [obj.messages_stored.vehicle_index] == ...
                int32(vehicle_index_subscribed);

            if ~any(is_found_message)
                return
            end

            % return latest message
            % the last message corresponds to the latest message
            % because new messages are always appended at the end of the queue
            latest_msg = obj.messages_stored( ...
                find(is_found_message, 1, "last") ...
            );
        end

        function tf = is_msg_outdated( ...
                obj, ...
                message_received ...
            )

            tf = ...
                ([obj.messages_stored.vehicle_index] == ...
                message_received.vehicle_index) & ...
                ([obj.messages_stored.time_step] == ...
                message_received.time_step);
        end

        function tf = is_found_message( ...
                obj, ...
                vehicle_index_subscribed, ...
                time_step, ...
                ~ ...
            )

            arguments
                obj (1, 1) InterHlcCommunication
                vehicle_index_subscribed (1, 1) double
                time_step (1, 1) double
                ~
            end

            tf = ...
                [obj.messages_stored.vehicle_index] == ...
                int32(vehicle_index_subscribed) & ...
                [obj.messages_stored.time_step] == ...
                int32(time_step);

        end

    end

end
