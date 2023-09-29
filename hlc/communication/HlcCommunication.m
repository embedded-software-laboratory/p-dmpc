classdef (Abstract) HlcCommunication < handle
    % abstract class for communication of a message

    properties
        ros2_node (1, 1) % ros2 node
        ros2_publisher (1, 1) % ros2 publisher
        ros2_subscriber (1, 1) % ros2 subscriber
        vehicle_id (1, 1) double % vehicle id
        topic_name (1, :) char % topic name in ros2 notation (with leading /)
        message_type (1, :) char % message type in ros2 notation
        message_to_be_sent (1, 1) struct % initialize message type
        messages_stored (1, :) struct % list with message structs
        options (1, 1) struct % options to create publisher and subscriber
    end

    methods (Abstract)
        send_message(obj)
    end

    methods

        function obj = HlcCommunication()
            % create communication class to connect to ROS2 network
        end

        function initialize(obj, vehicle_id, ...
                node_suffix, topic_name, message_type)

            arguments
                obj (1, 1) HlcCommunication
                vehicle_id (1, 1) double
                node_suffix (1, :) char
                topic_name (1, :) char
                message_type (1, :) char
            end

            % store vehicle id to associate a message
            % with the controlled vehicle
            obj.vehicle_id = vehicle_id;
            % store topic name to use it for
            % creation of publisher and subscriber
            obj.topic_name = topic_name;
            % store message type to use it for
            % creation of publisher and subscriber
            obj.message_type = message_type;
            % create ros2 node for communication
            obj.ros2_node = ros2node(['/node_', num2str(obj.vehicle_id), ...
                                          '_', node_suffix]);
            % create ros2 message structure
            % (faster than creating every time step)
            obj.message_to_be_sent = ros2message(obj.message_type);
            % store quality of service settings for publisher and subscriber
            obj.options = struct("History", "keeplast", "Depth", 40, ...
                "Durability", "transientlocal");
        end

        function create_publisher(obj)
            % workaround to be able to create publisher in the lab
            obj.ros2_publisher = ros2publisher(obj.ros2_node, ...
            "/parameter_events");
            % all vehicles share the same topic
            % (it is accepted that a vehicle is triggered by its own message)
            obj.ros2_publisher = ros2publisher(obj.ros2_node, ...
                obj.topic_name, obj.message_type, obj.options);
        end

        function create_subscriber(obj)
            % all vehicles share the same topic
            % (it is accepted that a vehicle is triggered by its own message)
            obj.ros2_subscriber = ros2subscriber(obj.ros2_node, ...
                obj.topic_name, obj.message_type, ...
                @obj.callback_subscriber, obj.options);
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

            if int32(obj.vehicle_id) == message_received.vehicle_id
                % if triggered by own message, do nothing
                return;
            end

            if isempty(obj.messages_stored)
                % if empty (no messages received so far)
                obj.messages_stored = message_received;
            else
                % else check if message with same vehicle id and time step
                % exists and delete this message then
                is_msg_outdated = ...
                    ([obj.messages_stored.vehicle_id] == ...
                    message_received.vehicle_id) & ...
                    ([obj.messages_stored.time_step] == ...
                    message_received.time_step);
                obj.messages_stored(is_msg_outdated) = [];

                % append message to list
                obj.messages_stored(end + 1) = message_received;
            end

            % delete messages older than a certain time steps compared to
            % the time step of newly received message
            message_age_threshold = 2;
            is_msg_expired = ...
                [obj.messages_stored.time_step] <= ...
                (message_received.time_step - message_age_threshold);
            obj.messages_stored(is_msg_expired) = [];
        end

        function latest_msg = read_message(obj, vehicle_id_subscribed, ...
                time_step, throw_error, timeout)
            % read message from the given time step

            arguments
                obj (1, 1) HlcCommunication
                vehicle_id_subscribed (1, 1) double
                time_step (1, 1) double
                throw_error (1, 1) logical = true
                timeout (1, 1) double = 100.0
            end

            is_timeout = true;
            % start timer for detecting timeout
            read_start = tic; read_time = toc(read_start);

            % initialize returned message
            % used if queue is empty or timeout without error
            latest_msg = struct([]);

            while read_time < timeout

                if ~isempty(obj.messages_stored)
                    % find messages by vehicle id and time step
                    is_found_message = ...
                        [obj.messages_stored.vehicle_id] == ...
                        int32(vehicle_id_subscribed) & ...
                        [obj.messages_stored.time_step] == ...
                        int32(time_step);

                    if any(is_found_message)
                        % only one message should be found
                        latest_msg = obj.messages_stored(is_found_message);
                        % if message is found timeout is not triggered
                        is_timeout = false;
                        break;
                    end

                end

                read_time = toc(read_start);
                % pause is necessary that MATLAB can executed the callback
                % function when the while loop is running
                pause(1e-4)
            end

            if is_timeout

                if throw_error
                    error(['Unable to receive the current message ', ...
                           'of step %d from vehicle %d within %d seconds'], ...
                        time_step, vehicle_id_subscribed, timeout)
                end

            end

        end

        function latest_msg = read_latest_message(obj, vehicle_id_subscribed)
            % read latest available message from the given subscriber

            arguments
                obj (1, 1) HlcCommunication
                vehicle_id_subscribed (1, 1) double
            end

            is_found = false;

            if ~isempty(obj.messages_stored)
                % find messages by vehicle id
                is_found_message = [obj.messages_stored.vehicle_id] == ...
                    int32(vehicle_id_subscribed);

                if any(is_found_message)
                    % return latest message
                    latest_msg = obj.messages_stored( ...
                        find(is_found_message, 1, "last"));
                    is_found = true;
                end

            end

            if ~is_found
                % return empty struct if no message is found
                latest_msg = struct([]);
            end

        end

    end

end
