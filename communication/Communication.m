classdef Communication
% SCENARIO  Communication class

    properties
        ros2_node;              % node of ROS 2
        vehicle_id;             % vehicle ID
        publisher;              % vehicle as publisher to send message
        time_step = int32(0);   % time step
        stored_msgs;            % stored messages
        msg_to_be_sent;                    % initialize message type
        options;                % options to create publisher and subscriber
    end
    
    properties (Dependent)

    end
    
    methods
        function obj = Communication()
            % Create node connected to ROS 2 network for communication
%             setenv("ROS_DOMAIN_ID","11") % change domain ID from default value 0 to 11. Vehicles can only communicate inside the same domain.
%             domain_ID = 11; % Vehicles can only communicate with vehicles in the same domain.
        end
        
        function obj = initialize_communication(obj, vehicle_id)
            obj.vehicle_id = vehicle_id;
            node_name = ['/node_',num2str(obj.vehicle_id)];
            obj.ros2_node = ros2node(node_name);
            obj.msg_to_be_sent = ros2message('veh_msgs/Traffic'); % create ROS 2 message structure
            obj.options = struct("History","keeplast","Depth",40,"Durability","transientlocal");
        end

        function obj = create_publisher(obj)
            % create publisher: each vehicle send message only to its own topic with name '/vehicle_ID'
            topic_name_publish = ['/vehicle_',num2str(obj.vehicle_id)]; 
            obj.publisher = ros2publisher(obj.ros2_node, topic_name_publish, "veh_msgs/Traffic", obj.options);
        end

        function ros_subscribers = create_subscriber(obj, vehs_to_be_subscribed)

            ros_subscribers = cell(length(vehs_to_be_subscribed), 1);
            % create subscribers: all topics should be subscribed
            for i = 1:length(vehs_to_be_subscribed)
                veh_id = vehs_to_be_subscribed(i);
                topic_name_subscribe = ['/vehicle_',num2str(veh_id)];
%                 callback = {@(msg) disp(msg)}; % callback function which will be executed automatically when receiving new message 
%                 obj.subscribe{iVeh} = ros2subscriber(obj.ros2_node,topic_name_subscribe,"veh_msgs/Traffic",@callback_when_receiving_message,options);
%                 obj.subscribe{iVeh} = ros2subscriber(obj.ros2_node, topic_name_subscribe, "veh_msgs/Traffic", options);
                ros_subscribers{i} = ros2subscriber(obj.ros2_node, topic_name_subscribe, "veh_msgs/Traffic", obj.options);
            end
        end

        function send_message(obj, time_step, predicted_trims, predicted_lanelets, predicted_areas)
            % vehicle send message to its topic
            obj.msg_to_be_sent.time_step = int32(time_step);
            obj.msg_to_be_sent.vehicle_id = int32(obj.vehicle_id);
            obj.msg_to_be_sent.predicted_trims = int32(predicted_trims(:));
            obj.msg_to_be_sent.predicted_lanelets = int32(predicted_lanelets(:));

            for i = 1:length(predicted_areas)
                obj.msg_to_be_sent.predicted_areas(i).x = predicted_areas{i}(1,:)';
                obj.msg_to_be_sent.predicted_areas(i).y = predicted_areas{i}(2,:)';
            end     

            % comment out if vehicles send their reachable sets to others
%             for j = 1:length(reachable_sets)
%                 msg.reachable_sets(j).x = vehicle.reachable_sets{j}.Vertices(:,1);
%                 msg.reachable_sets(j).y = vehicle.reachable_sets{j}.Vertices(:,2);
%             end

            send(obj.publisher, obj.msg_to_be_sent);
        end

        function latest_msg = read_message(~, sub, time_step)
            % Read message from the given time step
            timeout = 0.5;      is_timeout = true;
            read_start = tic;   read_time = toc(read_start);
            
            while read_time < timeout
                if sub.LatestMessage.time_step == time_step
                    disp(['Get current message after ' num2str(read_time) ' seconds.'])
                    is_timeout = false;
                    break
                end
                read_time = toc(read_start);
            end

            if is_timeout
                warning('Unable to receive the current message. The pevious message will be used.')
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
