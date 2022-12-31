classdef TrafficCommunication
% SCENARIO  Communication class

    properties
        ros2_node;              % node of ROS 2
        vehicle_id;             % vehicle ID
        publisher;              % vehicle as publisher to send message
        time_step = int32(0);   % time step
        stored_msgs;            % stored messages
        msg_to_be_sent;         % initialize message type
        options;                % options to create publisher and subscriber
        pose_indices;           % index of x, y, heading etc. in array
    end
    
    properties (Dependent)

    end
    
    methods
        function obj = TrafficCommunication()
            % Create node connected to ROS 2 network for communication
%             setenv("ROS_DOMAIN_ID","11") % change domain ID from default value 0 to 11. Vehicles can only communicate inside the same domain.
%             domain_ID = 11; % Vehicles can only communicate with vehicles in the same domain.
            obj.pose_indices = indices();
        end
        
        function obj = initialize_communication(obj, vehicle_id)
            obj.vehicle_id = vehicle_id;
            node_name = ['/node_',num2str(obj.vehicle_id)];
            obj.ros2_node = ros2node(node_name);
            obj.msg_to_be_sent = ros2message("veh_msgs/Traffic", "DataFormat","struct"); % create ROS 2 message structure
            obj.options = struct("DataFormat","struct","History","keeplast","Depth",40,"Reliability", "reliable", "Durability","transientlocal");
        end

        function obj = create_publisher(obj)
            % workaround to be able to create publisher in the lab
            obj.publisher = ros2publisher(obj.ros2_node,"/parameter_events");
            % create publisher: each vehicle send message only to its own topic with name '/vehicle_ID'
            topic_name_publish = ['/vehicle_',num2str(obj.vehicle_id), '_traffic']; 
            obj.publisher = ros2publisher(obj.ros2_node, topic_name_publish, "veh_msgs/Traffic", obj.options);
        end

        function ros_subscribers = create_subscriber(obj, vehs_to_be_subscribed)

            ros_subscribers = cell(length(vehs_to_be_subscribed), 1);
            % create subscribers: all topics should be subscribed
            for i = 1:length(vehs_to_be_subscribed)
                veh_id = vehs_to_be_subscribed(i);
                topic_name_subscribe = ['/vehicle_',num2str(veh_id), '_traffic'];
%                 callback = {@(msg) disp(msg)}; % callback function which will be executed automatically when receiving new message 
%                 obj.subscribe{iVeh} = ros2subscriber(obj.ros2_node,topic_name_subscribe,"veh_msgs/Traffic",@callback_when_receiving_message,options);
%                 obj.subscribe{iVeh} = ros2subscriber(obj.ros2_node, topic_name_subscribe, "veh_msgs/Traffic", options);
                ros_subscribers{i} = ros2subscriber(obj.ros2_node, topic_name_subscribe, "veh_msgs/Traffic", obj.options);
            end
        end

        function send_message(obj, time_step, current_pose, current_trim_index, predicted_lanelets, occupied_areas, reachable_sets, is_fallback)
            % vehicle send message to its topic
            obj.msg_to_be_sent.time_step = int32(time_step);
            obj.msg_to_be_sent.vehicle_id = int32(obj.vehicle_id);
            obj.msg_to_be_sent.current_pose.x = current_pose(obj.pose_indices.x);
            obj.msg_to_be_sent.current_pose.y = current_pose(obj.pose_indices.y);
            obj.msg_to_be_sent.current_pose.heading = current_pose(obj.pose_indices.heading);
            obj.msg_to_be_sent.current_pose.speed = current_pose(obj.pose_indices.speed);
            obj.msg_to_be_sent.current_trim_index = int32(current_trim_index);
            obj.msg_to_be_sent.predicted_lanelets = int32(predicted_lanelets);

            if nargin <= 7
                is_fallback = false;
            end
            
            obj.msg_to_be_sent.is_fallback = is_fallback; % whether vehicle should take fallback

            % occupied areas of current time step. normal offset
            % at index 1, without offset at index 2
            obj.msg_to_be_sent.occupied_areas(1).x = occupied_areas.normal_offset(1,:);
            obj.msg_to_be_sent.occupied_areas(1).y = occupied_areas.normal_offset(2,:);
            obj.msg_to_be_sent.occupied_areas(2).x = occupied_areas.without_offset(1,:);
            obj.msg_to_be_sent.occupied_areas(2).y = occupied_areas.without_offset(2,:);

            % comment out if vehicles send their reachable sets to others
            for i = 1:length(reachable_sets)
                obj.msg_to_be_sent.reachable_sets(i).x = reachable_sets{i}.Vertices(:,1);
                obj.msg_to_be_sent.reachable_sets(i).y = reachable_sets{i}.Vertices(:,2);
            end

            send(obj.publisher, obj.msg_to_be_sent);
        end

        function latest_msg = read_message(~, sub, time_step, timeout)
            % Read message from the given time step
            if nargin <= 3
                timeout = 2.0; 
            end
            is_timeout = true;
            read_start = tic;   read_time = toc(read_start);
            
            while read_time < timeout
                if ~isempty(sub.LatestMessage)
                    if sub.LatestMessage.time_step == time_step
                        % disp(['Get current message after ' num2str(read_time) ' seconds.'])
                        is_timeout = false;
                        break
                    elseif sub.LatestMessage.time_step > time_step
                        disp(['missed message ',num2str(time_step),'. Using ', num2str(sub.LatestMessage.time_step)])
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

%         function obj = get_stored_msgs(obj)
%             % get the stored messages
%             global stored_msgs_global
%             obj.stored_msgs = stored_msgs_global;
%         end

    end
    
    
end
