classdef Communication
% SCENARIO  Communication class

    properties
        ros2_node;              % node of ROS 2
        vehicle_id;             % vehicle ID
        publisher_a;            % vehicle as publisher to send message
        publisher_b;            % vehicle as publisher to send message
        time_step = int32(0);   % time step
        stored_msgs;            % stored messages
        msg_to_be_sent_a;       % initialize message type
        msg_to_be_sent_b;       % initialize message type
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
%             obj.msg_to_be_sent_a = ros2message('veh_msgs/Traffic'); % create ROS 2 message structure
%             obj.msg_to_be_sent_b = ros2message('std_msgs/Int32'); % create ROS 2 message structure
            obj.options = struct("History","keeplast","Depth",5,"Durability","transientlocal");
        end

        function obj = create_publisher(obj)
            % workaround to be able to create publisher in the lab
            obj.publisher_a = ros2publisher(obj.ros2_node,"/parameter_events");
            % create publisher: each vehicle send message only to its own topic with name '/vehicle_ID'

            % Main topic: predicted trajectory, trims, lanelets and so on
            topic_name_a = ['/vehicle_',num2str(obj.vehicle_id),'_a']; 
            % Subtopic: timestep at which all the latest messages from others
            % are read and not needed anymore so that others can feel free
            % to send their newest message (this precondition is to be
            % fulfilled because only the latest messages from others are
            % available. In other word, we do not need to save old
            % messages)
            topic_name_b = ['/vehicle_',num2str(obj.vehicle_id),'_b']; 

            [obj.publisher_a, obj.msg_to_be_sent_a] = ros2publisher(obj.ros2_node, topic_name_a, "veh_msgs/Traffic", obj.options);
            [obj.publisher_b, obj.msg_to_be_sent_b] = ros2publisher(obj.ros2_node, topic_name_b, "std_msgs/Int32", obj.options);
        end

        function [rosSubs_trafficInfo,rosSubs_timeStepAllMsgsAreRead] = create_subscriber(obj, vehs_to_be_subscribed)

            rosSubs_trafficInfo = cell(length(vehs_to_be_subscribed), 1);
            rosSubs_timeStepAllMsgsAreRead = cell(length(vehs_to_be_subscribed), 1);
            % create subscribers: all topics should be subscribed
            for i = 1:length(vehs_to_be_subscribed)
                veh_id = vehs_to_be_subscribed(i);
                topic_name_a = ['/vehicle_',num2str(veh_id),'_a']; % main topic: predicted trajectory, trims, lanelets and so on
                topic_name_b = ['/vehicle_',num2str(veh_id),'_b']; % subtopic: timestep at which all the latest messages from others are read and not needed anymore

%                 callback = {@(msg) disp(msg)}; % callback function which will be executed automatically when receiving new message 
%                 obj.subscribe{iVeh} = ros2subscriber(obj.ros2_node,topic_name_subscribe,"veh_msgs/Traffic",@callback_when_receiving_message,options);
%                 obj.subscribe{iVeh} = ros2subscriber(obj.ros2_node, topic_name_subscribe, "veh_msgs/Traffic", options);
                rosSubs_trafficInfo{i,1} = ros2subscriber(obj.ros2_node, topic_name_a, "veh_msgs/Traffic", obj.options);
                rosSubs_timeStepAllMsgsAreRead{i,1} = ros2subscriber(obj.ros2_node, topic_name_b, "std_msgs/Int32", obj.options);
            end
        end

        function send_message(obj, time_step, predicted_trims, predicted_lanelets, predicted_areas, is_fallback, vehs_fallback, states)
            % Send message of traffic information
            obj.msg_to_be_sent_a.time_step = int32(time_step);
            obj.msg_to_be_sent_a.vehicle_id = int32(obj.vehicle_id);
            obj.msg_to_be_sent_a.predicted_trims = int32(predicted_trims(:));
            obj.msg_to_be_sent_a.predicted_lanelets = int32(predicted_lanelets(:));

            if nargin <= 5
                is_fallback = false;
                vehs_fallback = []; % empty if no vehicle should take fallback
                states = [];
            end
            
            obj.msg_to_be_sent_a.is_fallback = is_fallback; % whether vehicle should take fallback
            obj.msg_to_be_sent_a.vehs_fallback = int32(vehs_fallback(:)); % which vehicles need to take fallback

            for i = 1:length(predicted_areas)
                obj.msg_to_be_sent_a.predicted_areas(i).x = predicted_areas{i}(1,:)';
                obj.msg_to_be_sent_a.predicted_areas(i).y = predicted_areas{i}(2,:)';
            end

            obj.msg_to_be_sent_a.states = states(:)'; % vehicle states [x;y;yaw;speed]

            % comment out if vehicles send their reachable sets to others
%             for j = 1:length(reachable_sets)
%                 msg.reachable_sets(j).x = vehicle.reachable_sets{j}.Vertices(:,1);
%                 msg.reachable_sets(j).y = vehicle.reachable_sets{j}.Vertices(:,2);
%             end

            send(obj.publisher_a, obj.msg_to_be_sent_a);
        end

        function sendMsg_timeStepAllMsgsAreRead(obj,time_step)
            % Send time step at which all the latest messages are read and thus
            % not needed anymore
            obj.msg_to_be_sent_b.data = int32(time_step);
            send(obj.publisher_b, obj.msg_to_be_sent_b);
        end

        function latest_msg = readMsg_trafficInfo(obj, sub, time_step, dt)
            % Read message of traffic information from the given time step
            if nargin==2
                dt = 0.2;
            end
            timeout = dt;

            is_timeout = true;
            read_start = tic;   read_time = toc(read_start);
            
            if isempty(sub.LatestMessage)
                obj.waitUntilNoEmptyMsg(sub)
            end

            while read_time < timeout
                if sub.LatestMessage.time_step == time_step
%                     disp(['Get current message after ' num2str(read_time) ' seconds.'])
                    is_timeout = false;
                    break
                end
                read_time = toc(read_start);
                pause(1e-4)
            end

            if is_timeout
                warning(['Unable to receive the message from vehicle ' num2str(sub.LatestMessage.vehicle_id) ' from time step ' num2str(time_step) '. The pevious message will be used.'])
            end

            % return the latest message
            latest_msg = sub.LatestMessage;
        end

        function synchronize(obj, subs, dt)
            % Synchronizes the networked control system according to the
            % timestamp of their messages send via ROS2
            if nargin==1
                dt = 0.2;
            end
            timeout = dt;

            syn_start = tic;

            % Firstly, wait until there exist messages from all
            % vehicles (useful for initial timestep)
            exist_any_empty_msg = any(cellfun(@(c) isempty(c.LatestMessage),subs));
            if exist_any_empty_msg
                obj.waitUntilNoEmptyMsg(subs)
            end
            
%             count = 1;

            while true             
                % Then check their timestamp
                time_steps = cellfun(@(c)[c.LatestMessage.data],subs);

                % Vehicle should wait as long as its time step to read all the latest
                % messages from others is not the smallest
                if min(time_steps) == time_steps(obj.vehicle_id)
                    disp(['All vehicles have read from others. Synchronization time: ' num2str(toc(syn_start)) ' seconds.'])
                    return
                end
                
                pause(1e-4)

                if toc(syn_start) >= timeout
                    disp(['Already synchronized for ' num2str(toc(syn_start)) ' seconds. Some vehicles still have not read from others.'])
                    pause(2)
%                     count = count*5;
                end
            end
        end

%         function obj = get_stored_msgs(obj)
%             % get the stored messages
%             global stored_msgs_global
%             obj.stored_msgs = stored_msgs_global;
%         end

    end

    methods(Static)


        function waitUntilNoEmptyMsg(subs)
            % wait for others to send initial messages
            has_disp = false;
            if ~iscell(subs)
                subs = {subs};
            end
            while true
                exist_any_empty_msg = any(cellfun(@(c) isempty(c.LatestMessage),subs));
                if exist_any_empty_msg
                    if ~has_disp
                        disp('Waiting for others to send initial messages...')
                        has_disp = true; % only disp once
                    end
                    pause(1e-4)
                    continue
                else
                    return
                end
            end
        end
    end

end
