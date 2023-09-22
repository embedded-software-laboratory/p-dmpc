classdef PredictionsCommunication < handle
    % SCENARIO  Communication class

    properties
        ros2_node; % node of ROS 2
        vehicle_id; % vehicle ID
        publisher; % vehicle as publisher to send message
        subscribers (1, :) % cell array with ros2 subscribers (entry of vehicle itself is always empty)
        time_step = int32(0); % time step
        stored_msgs; % stored messages
        msg_to_be_sent; % initialize message type
        options; % options to create publisher and subscriber
        stored_prediction_msgs (1, :) struct % list with message structs
    end

    properties (Dependent)

    end

    methods

        function obj = PredictionsCommunication()
            % Create node connected to ROS 2 network for communication
            %             setenv("ROS_DOMAIN_ID","11") % change domain ID from default value 0 to 11. Vehicles can only communicate inside the same domain.
            %             domain_ID = 11; % Vehicles can only communicate with vehicles in the same domain.
        end

        function initialize_communication(obj, vehicle_id)
            obj.vehicle_id = vehicle_id;
            node_name = ['/node_', num2str(obj.vehicle_id)];
            obj.ros2_node = ros2node(node_name);
            obj.msg_to_be_sent = ros2message('veh_msgs/Predictions'); % create ROS 2 message structure
            obj.options = struct("History", "keeplast", "Depth", 40, "Durability", "transientlocal");
        end

        function create_publisher(obj)
            % workaround to be able to create publisher in the lab
            obj.publisher = ros2publisher(obj.ros2_node, "/parameter_events");
            % create publisher: each vehicle send message only to its own topic with name '/vehicle_ID'
            topic_name_publish = ['/vehicle_', num2str(obj.vehicle_id), '_pred'];
            obj.publisher = ros2publisher(obj.ros2_node, topic_name_publish, "veh_msgs/Predictions", obj.options);
        end

        function create_subscriber(obj, veh_indices_to_be_subscribed, veh_ids_to_be_subscribed, amount)

            % initialize by constructing the last element
            obj.subscribers{amount} = [];

            for i = 1:length(veh_indices_to_be_subscribed)
                veh_id = veh_ids_to_be_subscribed(i);
                topic_name_subscribe = ['/vehicle_', num2str(veh_id), '_pred'];
                obj.subscribers{veh_indices_to_be_subscribed(i)} = ros2subscriber(obj.ros2_node, topic_name_subscribe, "veh_msgs/Predictions", @obj.callback_subscriber, obj.options);
            end

        end

        function callback_subscriber(obj, msg)
            % CALLBACK_WHEN_RECEIVING_MESSAGE This is a callback function and will be
            % executed automatically every time subscriber of ROS received a message.
            %
            % A global variable is defined to store message. Every time a message is
            % received, the publisher and the time step of the message will be checked.
            % A threshold of time step is defined for old messages. Too old messages
            % are deleted.

            if isempty(obj.stored_prediction_msgs)
                % if empty (no messages received so far)
                obj.stored_prediction_msgs = msg;
            else
                % else check if message with same vehicle id and time step exists
                % and delete this message then
                is_msg_outdated = ...
                    ([obj.stored_prediction_msgs.vehicle_id] == msg.vehicle_id) & ...
                    ([obj.stored_prediction_msgs.time_step] == msg.time_step);
                obj.stored_prediction_msgs(is_msg_outdated) = [];
                % append message to list
                obj.stored_prediction_msgs(end + 1) = msg;
            end

            % delete messages older than a certain time steps compared to the time step of newly received message
            message_age_threshold = 2;
            is_msg_expired = [obj.stored_prediction_msgs.time_step] <= (msg.time_step - message_age_threshold);
            obj.stored_prediction_msgs(is_msg_expired) = [];
        end

        function send_message(obj, time_step, predicted_areas, vehs_fallback)
            % vehicle send message to its topic
            obj.msg_to_be_sent.time_step = int32(time_step);
            obj.msg_to_be_sent.vehicle_id = int32(obj.vehicle_id);

            if nargin <= 3
                vehs_fallback = int32.empty();
            end

            obj.msg_to_be_sent.vehs_fallback = int32(vehs_fallback); % which vehicles should take fallback

            for i = 1:length(predicted_areas)
                obj.msg_to_be_sent.predicted_areas(i).x = predicted_areas{i}(1, :)';
                obj.msg_to_be_sent.predicted_areas(i).y = predicted_areas{i}(2, :)';
            end

            send(obj.publisher, obj.msg_to_be_sent);
        end

        function latest_msg = read_message(obj, sub, time_step, throw_error, timeout)
            % Read message from the given time step
            if nargin <= 4
                timeout = 100.0;
            end

            is_timeout = true;
            % start timer for detecting timeout
            read_start = tic; read_time = toc(read_start);

            % get id of the vehicle from which the message is read
            topic_name_split = split(sub.TopicName, '_');
            vehicle_id_subscribed = str2double(topic_name_split(2));

            while read_time < timeout

                if ~isempty(obj.stored_prediction_msgs)
                    % find messages by vehicle id and time step
                    is_found_message = ...
                        [obj.stored_prediction_msgs.vehicle_id] == int32(vehicle_id_subscribed) & ...
                        [obj.stored_prediction_msgs.time_step] == int32(time_step);

                    if any(is_found_message)
                        % only one message should be found
                        latest_msg = obj.stored_prediction_msgs(is_found_message);
                        % if message is found timeout is not triggered
                        is_timeout = false;
                        break;
                    end

                end

                read_time = toc(read_start);
                % pause is necessary that MATLAB can executed the callback function
                % when the while loop is running
                pause(1e-4)
            end

            if is_timeout

                if throw_error
                    error('Unable to receive the current message of step %i from vehicle %s within %d seconds', time_step, sub.TopicName, timeout)
                end

            end

        end

        function latest_msg = read_latest_message(obj, sub)
            % Read latest available message from the given subscriber

            is_found = false;

            % get id of the vehicle from which the message is read
            topic_name_split = split(sub.TopicName, '_');
            vehicle_id_subscribed = str2double(topic_name_split(2));

            if ~isempty(obj.stored_prediction_msgs)
                % find messages by vehicle id
                is_found_message = [obj.stored_prediction_msgs.vehicle_id] == int32(vehicle_id_subscribed);

                if any(is_found_message)
                    % return latest message
                    latest_msg = obj.stored_prediction_msgs(find(is_found_message, 1, "last"));
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
