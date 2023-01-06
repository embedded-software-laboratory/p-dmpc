function communication_init(hlc)
% COMMUNICATION_INIT This function initializes the communication network.
% ROS 2 nodes are created for each vehicle. Each vehicle has its own topic
% and sends its data only to its own topic.
%
% INPUT:
%   scenario: instance of the class Scenario
%
%   exp: object of the class 'SimLab' or 'CPMLab'
%
% OUTPUT:
%   scenario: instance of the class Scenario with instance of the class
%   Communication added to it

% generate custom message type (for vehicle communication) if not exist
msgList = ros2("msg","list"); % get all ROS 2 message types
[file_path,~,~] = fileparts(mfilename('fullpath'));
if (sum(cellfun(@(c)strcmp(c,'veh_msgs/Traffic'), msgList))==0) || (sum(cellfun(@(c)strcmp(c,'veh_msgs/Predictions'), msgList))==0)
    % if the message type 'veh_msgs/Traffic' does not exist
    path_custom_msg = [file_path,filesep,'cust1'];

    % Generate custom messages. NOTE that Python, CMake, and a C++ compiler are required (see
    % https://de.mathworks.com/help/ros/gs/ros-system-requirements.html
    % for more details according to your own MATLAB version).
    %
    % Useful functions:
    % 1. pyenv % check python version used by MATLAB
    % 2. pyenv('Version','requiredPythonVersionNumber') or pyenv('Version','fullPathOfPythonExe')
    % 3. !cmake --version % CMake version
    % 4. mex -setup % set c language compiler
    %
    % Note that sometimes ros2genmsg fails although all denpendencies
    % exist because the path where the custom messages are stored is
    % too deep. Try to move them to shallower path and try again.
    disp('Generating ROS 2 custom message type...')
    try
        ros2genmsg(path_custom_msg)
    catch ME
        disp(['If all environments for ros2genmsg() are prepared but still failed, try to move the whole folder to a ' ...
            'shallower path and run again if you use Windows machine, which sadly has a max path limit constraint.'])
        throw(ME)
    end
end

if sum(cellfun(@(c)strcmp(c,'ros_g29_force_feedback/ForceFeedback'), msgList))==0
    if ~hlc.scenario.options.is_sim_lab
        if ispc
            error('You are using a Windows machine, please do not select lab mode!')
        end
        % This message type is only needed for lab mode but not
        % simulation mode
        path_custom_msg = [file_path,filesep,'cust2'];
        disp('Generating ROS 2 custom message type...')
        ros2genmsg(path_custom_msg)
    end
end

nVeh = hlc.amount;
Hp = hlc.scenario.options.Hp;

% measure vehicles' initial poses and trims
[x0_measured, trims_measured] = hlc.hlc_adapter.measure();

% init global message storage for callbacks
% initialize the global variable
global stored_traffic_msgs;
stored_traffic_msgs = struct('time_step',[],'vehicle_id',[],'is_fallback',[],'current_pose',[],'current_trim_index',[],'predicted_lanelets',[],'occupied_areas',[],'reachable_sets',[], 'MessageType',[]);


%     topicList = ros2("topic","list");
%     nodeList = ros2("node","list");
if isempty(hlc.scenario.vehicles(hlc.indices_in_vehicle_list(1)).communicate)
    start = tic;
    disp('Creating ROS 2 publishers...')
    for index = hlc.indices_in_vehicle_list
        hlc.scenario.vehicles(index).communicate.traffic = TrafficCommunication(); % create instance of the Comunication class
        hlc.scenario.vehicles(index).communicate.traffic = initialize_communication(hlc.scenario.vehicles(index).communicate.traffic, hlc.scenario.vehicles(index).ID); % initialize
        hlc.scenario.vehicles(index).communicate.traffic = create_publisher(hlc.scenario.vehicles(index).communicate.traffic); % create publisher

        hlc.scenario.vehicles(index).communicate.predictions = PredictionsCommunication(); % create instance of the Comunication class
        hlc.scenario.vehicles(index).communicate.predictions = initialize_communication(hlc.scenario.vehicles(index).communicate.predictions, hlc.scenario.vehicles(index).ID); % initialize
        hlc.scenario.vehicles(index).communicate.predictions = create_publisher(hlc.scenario.vehicles(index).communicate.predictions); % create publisher
    end
end

% Create subscribers.
% Each vehicle subscribes all other vehicles.
% NOTE that subscribers are create only once but not loopover all
% vehicles to let all of them subscribe others because it is
% time-consuming to create many subscribers.
% The subscribers will be used by all vehicles.
disp('Creating ROS 2 subscribers...')
vehs_to_be_subscribed = hlc.scenario.options.veh_ids;
hlc.ros_subscribers.traffic = create_subscriber(hlc.scenario.vehicles(hlc.indices_in_vehicle_list(1)).communicate.traffic,vehs_to_be_subscribed);
hlc.ros_subscribers.predictions = create_subscriber(hlc.scenario.vehicles(hlc.indices_in_vehicle_list(1)).communicate.predictions,vehs_to_be_subscribed);
if length(hlc.indices_in_vehicle_list) == 1
    pause(5.0) % wait for all subscribers to be created
end
duration = toc(start);
disp(['Finished in ' num2str(duration) ' seconds.'])

if ~hlc.scenario.options.is_mixed_traffic
    % Communicate predicted trims, pridicted lanelets and areas to other vehicles
    for veh_index = hlc.indices_in_vehicle_list
        predicted_trims = repmat(trims_measured(veh_index), 1, Hp+1); % current trim and predicted trims in the prediction horizon

        x0 = x0_measured(veh_index,indices().x); % vehicle position x
        y0 = x0_measured(veh_index,indices().y); % vehicle position y
        heading = x0_measured(veh_index,indices().heading);
        speed = x0_measured(veh_index,indices().speed);
        current_pose = [x0,y0,heading,speed];

        predicted_lanelets = get_predicted_lanelets(hlc.scenario,veh_index,x0,y0);

        % get vehicles currently occupied area
        x_rec1 = [-1, -1,  1,  1, -1] * (hlc.scenario.vehicles(veh_index).Length/2 + hlc.scenario.options.offset); % repeat the first entry to enclose the shape
        y_rec1 = [-1,  1,  1, -1, -1] * (hlc.scenario.vehicles(veh_index).Width/2 + hlc.scenario.options.offset);
        % calculate displacement of model shape
        [x_rec2, y_rec2] = translate_global(heading, x0, y0, x_rec1, y_rec1);
        occupied_area.normal_offset = [x_rec2; y_rec2];

        x_rec1_without_offset = [-1, -1,  1,  1, -1] * (hlc.scenario.vehicles(veh_index).Length/2); % repeat the first entry to enclose the shape
        y_rec1_without_offset = [-1,  1,  1, -1, -1] * (hlc.scenario.vehicles(veh_index).Width/2);
        [x_rec2_without_offset, y_rec2_without_offset] = translate_global(heading, x0, y0, x_rec1_without_offset, y_rec1_without_offset);
        occupied_area.without_offset = [x_rec2_without_offset; y_rec2_without_offset];

        predicted_occupied_areas = {}; % for initial time step, the occupied areas are not predicted yet
        reachable_sets = {}; % for initial time step, the reachable sets are not computed yet
        hlc.scenario.vehicles(veh_index).communicate.predictions.send_message(hlc.scenario.k, predicted_trims, predicted_lanelets, predicted_occupied_areas);
        hlc.scenario.vehicles(veh_index).communicate.traffic.send_message(hlc.scenario.k, current_pose, predicted_trims(1), predicted_lanelets, occupied_area, reachable_sets);
    end
    % read form all other vehicles to make sure all vehicles are ready
    other_vehicles = setdiff(1:hlc.scenario.options.amount, hlc.indices_in_vehicle_list);
    for veh_index = other_vehicles
        disp(['reading initial msg from vehicle ', num2str(veh_index)]);
        read_message(hlc.scenario.vehicles(hlc.indices_in_vehicle_list(1)).communicate.traffic, hlc.ros_subscribers.traffic{veh_index}, hlc.scenario.k, 10.0);

        %         timeout = 10.0;
        %         time_step = 0;
        %         is_timeout = true;
        %         read_start = tic;   read_time = toc(read_start);
        %         while read_time < timeout
        %             if ~isempty(hlc.ros_subscribers.traffic{veh_index}.LatestMessage)
        %                 if hlc.ros_subscribers.traffic{veh_index}.LatestMessage.time_step == time_step
        %                     % disp(['Get current message after ' num2str(read_time) ' seconds.'])
        %                     is_timeout = false;
        %                     break
        %                 elseif sub.LatestMessage.time_step > time_step
        %                     disp(['missed message ',num2str(time_step),'. Using ', num2str(hlc.ros_subscribers.traffic{veh_index}.LatestMessage.time_step)])
        %                     is_timeout = false;
        %                     break
        %                 end
        %             end
        %             read_time = toc(read_start);
        %             pause(1e-2);
        %         end
    end
    pause(1.2);
end
end

