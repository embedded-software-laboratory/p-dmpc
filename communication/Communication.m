classdef Communication
% SCENARIO  Communication class

    properties
        ros2_node;              % node of ROS 2
        vehicle_id;             % vehicle ID
        pub_trafficInfo;            % publish traffic information (after graph search)
        pub_beginingEachTimeStep    % publish the current time step at the begining of each time step
        pub_timeStepAllMsgsAreRead; % publish the current time step immediately after all latest traffic information messages from others are read during each time step 
        
        publisher_c;            % vehicle as publisher to send message
        time_step = int32(0);   % time step
        stored_msgs;            % stored messages
        msg_trafficInfo;       % initialize message type
        msg_beginingEachTimeStep;       % initialize message type
        msg_timeStepAllMsgsAreRead;       % initialize message type
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
            obj.pub_trafficInfo = ros2publisher(obj.ros2_node,"/parameter_events");
            % create publisher: each vehicle send message only to its own topic with name '/vehicle_ID'

            % Main topic: predicted trajectory, trims, lanelets and so on
            topic_name_a = ['/vehicle_',num2str(obj.vehicle_id),'_trafficInfo']; 
            % Subtopic: timestep at which all the latest messages from others
            % are read and not needed anymore so that others can feel free
            % to send their newest message (this precondition is to be
            % fulfilled because only the latest messages from others are
            % available. In other word, we do not need to save old
            % messages)
            topic_name_b = ['/vehicle_',num2str(obj.vehicle_id),'_timeStepAllMsgsAreRead']; 
            topic_name_c = ['/vehicle_',num2str(obj.vehicle_id),'_beginingEachTimeStep']; 

            [obj.pub_trafficInfo, obj.msg_trafficInfo] = ros2publisher(obj.ros2_node, topic_name_a, "veh_msgs/Traffic", obj.options);
            [obj.pub_timeStepAllMsgsAreRead, obj.msg_beginingEachTimeStep] = ros2publisher(obj.ros2_node, topic_name_b, "std_msgs/Int32", obj.options);
            [obj.pub_beginingEachTimeStep, obj.msg_timeStepAllMsgsAreRead] = ros2publisher(obj.ros2_node, topic_name_c, "std_msgs/Int32", obj.options);
        end

        function [rosSubs_trafficInfo,rosSubs_timeStepAllMsgsAreRead,rosSubs_beginingEachTimeStep] = create_subscriber(obj, vehs_to_be_subscribed)
            num_subs = length(vehs_to_be_subscribed);
            rosSubs_trafficInfo = cell(num_subs, 1);
            rosSubs_timeStepAllMsgsAreRead = cell(num_subs, 1);
            rosSubs_beginingEachTimeStep = cell(num_subs, 1);

            % create subscribers: all topics should be subscribed
            for i = 1:num_subs
                veh_id = vehs_to_be_subscribed(i);
                topic_name_a = ['/vehicle_',num2str(veh_id),'_trafficInfo'];
                topic_name_b = ['/vehicle_',num2str(veh_id),'_timeStepAllMsgsAreRead']; 
                topic_name_c = ['/vehicle_',num2str(veh_id),'_beginingEachTimeStep']; 
%                 callback = {@(msg) disp(msg)}; % callback function which will be executed automatically when receiving new message 
%                 obj.subscribe{iVeh} = ros2subscriber(obj.ros2_node,topic_name_subscribe,"veh_msgs/Traffic",@callback_when_receiving_message,options);
%                 obj.subscribe{iVeh} = ros2subscriber(obj.ros2_node, topic_name_subscribe, "veh_msgs/Traffic", options);
                rosSubs_trafficInfo{i,1} = ros2subscriber(obj.ros2_node, topic_name_a, "veh_msgs/Traffic", obj.options);
                rosSubs_timeStepAllMsgsAreRead{i,1} = ros2subscriber(obj.ros2_node, topic_name_b, "std_msgs/Int32", obj.options);
                rosSubs_beginingEachTimeStep{i,1} = ros2subscriber(obj.ros2_node, topic_name_c, "std_msgs/Int32", obj.options);
            end
        end

        function sendMsg_trafficInfo(obj, time_step, predicted_trims, predicted_lanelets, predicted_areas, is_fallback, vehs_fallback, states)
            % Send message of traffic information
            obj.msg_trafficInfo.time_step = int32(time_step);
            obj.msg_trafficInfo.vehicle_id = int32(obj.vehicle_id);
            obj.msg_trafficInfo.predicted_trims = int32(predicted_trims(:));
            obj.msg_trafficInfo.predicted_lanelets = int32(predicted_lanelets(:));

            if nargin <= 5
                is_fallback = false;
                vehs_fallback = []; % empty if no vehicle should take fallback
                states = [];
            end
            
            obj.msg_trafficInfo.is_fallback = is_fallback; % whether vehicle should take fallback
            obj.msg_trafficInfo.vehs_fallback = int32(vehs_fallback(:)); % which vehicles need to take fallback

            for i = 1:length(predicted_areas)
                obj.msg_trafficInfo.predicted_areas(i).x = predicted_areas{i}(1,:)';
                obj.msg_trafficInfo.predicted_areas(i).y = predicted_areas{i}(2,:)';
            end

            obj.msg_trafficInfo.states = states(:)'; % vehicle states [x;y;yaw;speed]

            % comment out if vehicles send their reachable sets to others
%             for j = 1:length(reachable_sets)
%                 msg.reachable_sets(j).x = vehicle.reachable_sets{j}.Vertices(:,1);
%                 msg.reachable_sets(j).y = vehicle.reachable_sets{j}.Vertices(:,2);
%             end

            send(obj.pub_trafficInfo, obj.msg_trafficInfo);
        end

        function sendMsg_timeStepAllMsgsAreRead(obj,time_step)
            % Send the current time step (should be sent immediately after
            % all the latest traffic information messages from others are
            % read and thus are not needed anymore)
            obj.msg_timeStepAllMsgsAreRead.data = int32(time_step);
            send(obj.pub_timeStepAllMsgsAreRead, obj.msg_timeStepAllMsgsAreRead);
        end

        function sendMsg_beginingEachTimeStep(obj,time_step)
            % Send the current time step (should be sent at the begining of
            % each time step)
            obj.msg_beginingEachTimeStep.data = int32(time_step);
            send(obj.pub_beginingEachTimeStep, obj.msg_beginingEachTimeStep);
        end

        function latest_msg = readMsg_trafficInfo(obj, sub, time_step, dt)
            % Read message of traffic information from the given time step
            if nargin==2
                dt = 0.2;
            end
            timeout = 10;

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
            if nargin <= 2
                dt = 0.2;
            end
            timeout = dt;
            has_printed_msg = false;

            syn_start = tic;

            % Firstly, wait until there exist messages from all
            % vehicles (useful for initial timestep)
            exist_any_empty_msg = any(cellfun(@(c) isempty(c.LatestMessage),subs));
            if exist_any_empty_msg
                obj.waitUntilNoEmptyMsg(subs)
            end
            
            while true             
                % Then check their timestamp
                time_steps = cellfun(@(c)[c.LatestMessage.data],subs);

                % Vehicle should wait as long as its time step to read all the latest
                % messages from others is not the smallest
                if min(time_steps) == time_steps(obj.vehicle_id)
                    delayed_steps = max(time_steps) - min(time_steps);
                    if delayed_steps == 0
                        disp(['All vehicles are at the begining of time step ' num2str(min(time_steps)) '. Synchronization time: ' num2str(toc(syn_start)) ' seconds.'])
                    else
                        disp(['The ego vehicle is delayed for ' num2str(delayed_steps) ' time step(s). Synchronization time: ' num2str(toc(syn_start)) ' seconds.'])
                    end

                    return
                end

                if toc(syn_start) >= timeout
                    if ~has_printed_msg
                        disp(['Already synchronized for ' num2str(toc(syn_start)) ' seconds. Some vehicles are still in the old time step...'])
                        has_printed_msg = true; % only print once
                    end
                end

                pause(1e-4)
            end
        end

        function waitUntilOthersFinishReading(obj, subs, dt)
            % Wait until all vehicles have read the latest messages sent by
            % others
            if nargin==1
                dt = 0.2;
            end
            timeout = dt;
            has_printed_msg = false;

            syn_start = tic;

            % Firstly, wait until there exist messages from all
            % vehicles (useful for initial timestep)
            exist_any_empty_msg = any(cellfun(@(c) isempty(c.LatestMessage),subs));
            if exist_any_empty_msg
                obj.waitUntilNoEmptyMsg(subs)
            end
            
            while true             
                % Then check their timestamp
                time_steps = cellfun(@(c)[c.LatestMessage.data],subs);

                % Vehicle should wait as long as its time step to read all the latest
                % messages from others is not the smallest
                if min(time_steps) == time_steps(obj.vehicle_id)
                    disp(['All vehicles have read from others. Synchronization time: ' num2str(toc(syn_start)) ' seconds.'])
                    return
                end

                if toc(syn_start) >= timeout
                    if ~has_printed_msg
                        disp(['Already synchronized for ' num2str(toc(syn_start)) ' seconds. Waiting for others to read unread messages...'])
                        has_printed_msg = true; % only print once
                    end
                end

                pause(1e-4)
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
