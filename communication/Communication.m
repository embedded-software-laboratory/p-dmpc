classdef Communication
% SCENARIO  Communication class

    properties
        state = [-1 -1 -1 -1 -1]; % state [x-position y-position yaw velocity steering_angle]
        areas = {}; % predicted occupied areas
        trim_current = -1; % current trim
        lanelet_current = -1; % current lanelet in which the vehicle drives
        lanelets_future = []; % future lanelets that the vehicle plan to drive in
        ros2_node; % node of ROS 2
        veh_ID = -1; % vehicle ID
        publish; % vehicle itself as publisher of ROS 2
        subscribe; % subscribe other vehicles

    end
    
    properties (Dependent)

    end
    
    methods
        function obj = Communication(vehicle)
            % Create node connected to ROS 2 network for communication
%             setenv("ROS_DOMAIN_ID","11") % change domain ID from default value 0 to 11. Vehicles can only communicate inside the same domain.
%             domain_ID = 11; % Vehicles can only communicate with vehicles in the same domain.
            node_name = ['/node_',num2str(vehicle.ID)];
            obj.ros2_node = ros2node(node_name);

            v0 = 0; % initial speed
            gamma0 = 0; % initial steering angle
            obj.state = [vehicle.x_start vehicle.y_start vehicle.yaw_start v0 gamma0];

            obj.veh_ID = vehicle.ID;
        end
        
        function initialize(obj)
            disp('send init data')
        end

        function obj = create_pub_and_subs(obj,nVeh)
            % Create publisher and subscribers
            topic_name_publish = ['/vehicle_',num2str(obj.veh_ID)]; % each vehicle send message only to its own topic with name 'topic_vehicleID'
            obj.publish = ros2publisher(obj.ros2_node,topic_name_publish,"veh_msgs/Traffic"); % publish messages on a topic
            for iVeh=1:nVeh % each vehicle subscribes all other vehicles' topics; to simplify implementation, vehicle subscribe itself
                topic_name_subscribe = ['/vehicle_',num2str(iVeh)];
                obj.subscribe{iVeh} = ros2subscriber(obj.ros2_node,topic_name_subscribe,"veh_msgs/Traffic");
            end
        end

        function send_data(obj)
            % vehicle send its data to its topic
            msg = ros2message('veh_msgs/Traffic');
            msg.state = obj.state';
            for i=1:length(obj.areas)
                msg.areas(i).x = obj.areas{i}(1,:)';
                msg.areas(i).y = obj.areas{i}(2,:)';
            end
            msg.trim_current = int32(obj.trim_current);
            msg.veh_id = int32(obj.veh_ID);
            msg.lanelet_current = int32(obj.lanelet_current);
            msg.lanelets_future = int32(obj.lanelets_future');
            send(obj.publish,msg);
        end

    end
    
    
end
