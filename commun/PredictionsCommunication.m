classdef PredictionsCommunication
% SCENARIO  Communication class

    properties
        ros2_node;              % node of ROS 2
        vehicle_id;             % vehicle ID
        publisher;              % vehicle as publisher to send message
        time_step = int32(0);   % time step
        stored_msgs;            % stored messages
        msg_to_be_sent;         % initialize message type
        options;                % options to create publisher and subscriber
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
            obj.vehicle_id = vehicle_id;
            node_name = ['/node_',num2str(obj.vehicle_id)];
            obj.ros2_node = ros2node(node_name);
            obj.msg_to_be_sent = ros2message('veh_msgs/Predictions'); % create ROS 2 message structure
            obj.options = struct("History","keeplast","Depth",40,"Durability","transientlocal");
        end

        function obj = create_publisher(obj)
            % workaround to be able to create publisher in the lab
            obj.publisher = ros2publisher(obj.ros2_node,"/parameter_events");
            % create publisher: each vehicle send message only to its own topic with name '/vehicle_ID'
            topic_name_publish = ['/vehicle_',num2str(obj.vehicle_id), '_pred']; 
            obj.publisher = ros2publisher(obj.ros2_node, topic_name_publish, "veh_msgs/Predictions", obj.options);
        end

        function ros_subscribers = create_subscriber(obj, vehs_to_be_subscribed)

            ros_subscribers = cell(length(vehs_to_be_subscribed), 1);
            % create subscribers: all topics should be subscribed
            for i = 1:length(vehs_to_be_subscribed)
                veh_id = vehs_to_be_subscribed(i);
                topic_name_subscribe = ['/vehicle_',num2str(veh_id), '_pred'];
                ros_subscribers{i} = ros2subscriber(obj.ros2_node, topic_name_subscribe, "veh_msgs/Predictions", obj.options);
            end
        end

        function send_message(obj, time_step, predicted_trims, predicted_lanelets, predicted_areas, vehs_fallback)
            % vehicle send message to its topic
            obj.msg_to_be_sent.time_step = int32(time_step);
            obj.msg_to_be_sent.vehicle_id = int32(obj.vehicle_id);
            obj.msg_to_be_sent.predicted_trims = int32(predicted_trims(:));
            obj.msg_to_be_sent.predicted_lanelets = int32(predicted_lanelets(:));

            if nargin <= 5
                vehs_fallback = int32.empty();
            end
            
            obj.msg_to_be_sent.vehs_fallback = vehs_fallback; % which vehicles should take fallback

            for i = 1:length(predicted_areas)
                obj.msg_to_be_sent.predicted_areas(i).x = predicted_areas{i}(1,:)';
                obj.msg_to_be_sent.predicted_areas(i).y = predicted_areas{i}(2,:)';
            end

            send(obj.publisher, obj.msg_to_be_sent);
        end

        function latest_msg = read_message(~, sub, time_step, timeout)
            % Read message from the given time step
            if nargin <= 3
                timeout = 0.5; 
            end
            is_timeout = true;
            read_start = tic;   read_time = toc(read_start);
            
            while read_time < timeout
                if ~isempty(sub.LatestMessage)
                    if sub.LatestMessage.time_step == time_step
                        %                     disp(['Get current message after ' num2str(read_time) ' seconds.'])
                        is_timeout = false;
                        break
                    end
                end
                read_time = toc(read_start);
                pause(1e-4)
            end

            if is_timeout
                warning(['Unable to receive the current message of step %i from vehicle %s. The pevious message from step ' ...
                    '%i will be used.'], time_step, sub.TopicName, sub.LatestMessage.time_step)
            end

            % return the latest message
            latest_msg = sub.LatestMessage;
        end
    end   
end
