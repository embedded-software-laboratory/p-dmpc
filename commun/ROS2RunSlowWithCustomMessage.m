%% Generate custom message type (if not exist)
% Please do not use ctrl+enter or click on "Run Section", press F5 or click
% on "Run" instead (to make sure that the function `fileparts()` below can
% get the right path of the folder where the custom message is stored.)
msgList = ros2("msg","list"); % get all ROS 2 message types

% Custom message type
custom_msg_type = "veh_msgs/Traffic";
if sum(cellfun(@(c)strcmp(c,custom_msg_type), msgList))==0
    % if the message type 'veh_msgs/Traffic' does not exist
    [file_path,~,~] = fileparts(mfilename('fullpath'));
    path_custom_msg = [file_path,filesep,'custom'];
    
    disp('Generating ROS 2 custom message type...')
    try
        ros2genmsg(path_custom_msg)
    catch ME
        disp(['If all environments for ros2genmsg() are prepared but it still failed, try to move the whole folder to a ' ...
            'shallower path and run again if you use Windows machine, which sadly has a maximum path limit constraint.'])
        throw(ME)
    end
else
    disp('Message type already exists.')
end

%% A MWE to show that ROS 2 runs slow with custom message

% We have 20 vehicles. Each vehicle corresponds to a ROS 2 node,
% publishes an unique topic, and subscribes the topics of all other
% vehicles. To reduce the time of creating subscribers, only one vehicle
% subscribes others, and the resulting subscribers will be used by all
% vehicles. In summary, we need to create 20 nodes, 20 publishers, and 20
% subscribers. You can use ctrl+enter to run this section.

num_vehicles = 20;

nodes = cell(num_vehicles,1);
publishers = cell(num_vehicles,1);
subscribers = cell(num_vehicles,1);

% Custom message type
custom_msg_type = "veh_msgs/Traffic";

% Create ROS 2 nodes
start_create_nodes = tic;
disp('Creating ROS 2 nodes...')
for i = 1:num_vehicles
    name = ['/veh_' num2str(i)]; % node or topic name: /veh_1, /veh_2,..., /veh_20
    nodes{i} = ros2node(name);
end
end_create_nodes = toc(start_create_nodes);
disp(['Create 20 nodes took ' num2str(end_create_nodes) ' seconds.'])

% Create ROS 2 publishers
start_create_pubs = tic;
disp('Creating ROS 2 publishers...')
for i = 1:num_vehicles
    name = ['/veh_' num2str(i)]; % node or topic name: /veh_1, /veh_2,..., /veh_20
    publishers{i} = ros2publisher(nodes{i},name,custom_msg_type);
end
end_create_pubs = toc(start_create_pubs);
disp(['Create 20 publishers took ' num2str(end_create_pubs) ' seconds.'])

% Create ROS 2 subscribers
start_create_subs = tic;
disp('Creating ROS 2 subscribers...')
for i = 1:num_vehicles
    name = ['/veh_' num2str(i)]; % node or topic name: /veh_1, /veh_2,..., /veh_20
    subscribers{i} = ros2subscriber(nodes{i},name,custom_msg_type);
end
end_create_subs = toc(start_create_subs);
disp(['Create 20 subscribers took ' num2str(end_create_subs) ' seconds.'])

disp(['--------Totally took ' num2str(end_create_nodes+end_create_pubs+end_create_subs) ' seconds--------'])

% Results: 
% In my PC (MATLAB R2022a, windows 10 pro, 21H2 with AMD Ryzen 5700G core),
% creating 20 noded took 6 seconds, creating 20 publishers took 9 seconds,
% and creating 20 subscribers took also about 9 seconds. Therefore, totally
% about 24 seconds were needed, which is annoying as you need to wait for
% almost a half minute. 

