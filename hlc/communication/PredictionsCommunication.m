classdef PredictionsCommunication
    % SCENARIO  Communication class

    properties
        ros2_node; % node of ROS 2
        vehicle_id; % vehicle ID
        publisher; % vehicle as publisher to send message
        time_step = int32(0); % time step
        stored_msgs; % stored messages
        msg_to_be_sent; % initialize message type
        options; % options to create publisher and subscriber
    end

    properties (Dependent)

    end

    methods

        function obj = PredictionsCommunication()
            % Create node connected to ROS 2 network for communication
            %             setenv("ROS_DOMAIN_ID","11") % change domain ID from default value 0 to 11. Vehicles can only communicate inside the same domain.
            %             domain_ID = 11; % Vehicles can only communicate with vehicles in the same domain.
        end

        function obj = initialize_communication(obj, vehicle_id)
            % initialize empty message list, relevant if global variables are not cleared
            global stored_prediction_msgs
            stored_prediction_msgs = [];
            obj.vehicle_id = vehicle_id;
            node_name = ['/node_', num2str(obj.vehicle_id)];
            obj.ros2_node = ros2node(node_name);
            obj.msg_to_be_sent = ros2message('veh_msgs/Predictions'); % create ROS 2 message structure
            obj.options = struct("History", "keeplast", "Depth", 40, "Durability", "transientlocal");
        end

        function obj = create_publisher(obj)
            % workaround to be able to create publisher in the lab
            obj.publisher = ros2publisher(obj.ros2_node, "/parameter_events");
            % create publisher: each vehicle send message only to its own topic with name '/vehicle_ID'
            topic_name_publish = ['/vehicle_', num2str(obj.vehicle_id), '_pred'];
            obj.publisher = ros2publisher(obj.ros2_node, topic_name_publish, "veh_msgs/Predictions", obj.options);
        end

        function ros_subscribers = create_subscriber(obj, veh_indices_to_be_subscribed, veh_ids_to_be_subscribed, amount)

            ros_subscribers = cell(amount, 1);

            for i = 1:length(veh_indices_to_be_subscribed)
                veh_id = veh_ids_to_be_subscribed(i);
                topic_name_subscribe = ['/vehicle_', num2str(veh_id), '_pred'];
                ros_subscribers{veh_indices_to_be_subscribed(i)} = ros2subscriber(obj.ros2_node, topic_name_subscribe, "veh_msgs/Predictions", @obj.callback_subscriber, obj.options);
            end

        end

        function callback_subscriber(~, msg)
            % CALLBACK_WHEN_RECEIVING_MESSAGE This is a callback function and will be
            % executed automatically every time subscriber of ROS received a message.
            %
            % A global variable is defined to store message. Every time a message is
            % received, the publisher and the time step of the message will be checked.
            % A threshold of time step is defined for old messages. Too old messages
            % are deleted.

            global stored_prediction_msgs

            if isempty(stored_prediction_msgs)
                % if empty (no messages received so far)
                stored_prediction_msgs = msg;
            else
                % else check if message with same vehicle id and time step exists
                % and delete this message then
                is_msg_outdated = ...
                    ([stored_prediction_msgs.vehicle_id] == msg.vehicle_id) & ...
                    ([stored_prediction_msgs.time_step] == msg.time_step);
                stored_prediction_msgs(is_msg_outdated) = [];
                % append message to list
                stored_prediction_msgs(end + 1) = msg;
            end

            % delete messages older than a certain time steps compared to the time step of newly received message
            message_age_threshold = 2;
            is_msg_expired = [stored_prediction_msgs.time_step] <= (msg.time_step - message_age_threshold);
            stored_prediction_msgs(is_msg_expired) = [];
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

        function latest_msg = read_message(~, sub, time_step, throw_error, timeout)
            % Read message from the given time step
            if nargin <= 4
                timeout = 100.0;
            end

            is_timeout = true;
            read_start = tic; read_time = toc(read_start);

            while read_time < timeout

                if ~isempty(sub.LatestMessage)

                    if sub.LatestMessage.time_step == time_step
                        is_timeout = false;
                        break
                    end

                end

                read_time = toc(read_start);
                pause(1e-4)
            end

            if is_timeout

                if throw_error
                    error('Unable to receive the current message of step %i from vehicle %s within %d seconds', time_step, sub.TopicName, timeout)
                end

            end

            % return the latest message
            latest_msg = sub.LatestMessage;
        end

    end

end
