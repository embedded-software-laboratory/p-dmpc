function scenario = communication_init(scenario, exp)
% COMMUNICATION_INIT This function initializes the communication network.
% ROS 2 nodes are created for each vehicle. Each vehicle has its own topic
% and sends its data only to its own topic.
% 
% INPUT:
%   scenario: instance of the class Scenario
%   
%   iter: iteration information
% 
% OUTPUT:
%   scenario: instance of the class Scenario with instance of the class
%   Communication added to it


    % generate custom message type (for vehicle communication) if not exist
    msgList = ros2("msg","list"); % get all ROS 2 message types
    if sum(cellfun(@(c)strcmp(c,'veh_msgs/Traffic'), msgList))==0
        % if the message type 'veh_msgs/Traffic' does not exist 
        [file_path,~,~] = fileparts(mfilename('fullpath'));
        path_custom_msg = [file_path,filesep,'custom_msg'];
        
        % Generate custom messages. NOTE that Python, CMake, and a C++
        % compiler are required (see
        % https://de.mathworks.com/help/ros/gs/ros-system-requirements.html
        % for more details according to your MATLAB version).
        ros2genmsg(path_custom_msg) 
    end

    nVeh = scenario.nVeh;
    Hp = scenario.Hp;

    % measure vehicles' initial poses and trims
    [x0_measured, trims_measured] = exp.measure(false);

    for iVeh = 1:nVeh
        scenario.vehicles(iVeh).communicate = Communication(); % create instance of the Comunication class
        scenario.vehicles(iVeh).communicate = initialize_communication(scenario.vehicles(iVeh).communicate, scenario.vehicles(iVeh).ID); % initialize
        scenario.vehicles(iVeh).communicate = create_publisher(scenario.vehicles(iVeh).communicate, scenario); % create publisher
    end
    
    % Create subscribers.
    % Each vehicle subscribes all other vehicles.
    % NOTE that subscribers are create only once but not loopover all
    % vehicles to let all of them subscribe others because it is
    % time-consuming to create many subscribers. 
    % The subscribers will be used by all vehicles.
    vehs_to_be_subscribed = [scenario.vehicles.ID];
    scenario.ros_subscribers = scenario.vehicles(1).communicate.create_subscriber(vehs_to_be_subscribed); 

    if scenario.options.is_sim_lab
        % Communicate predicted trims, pridicted lanelets and areas to other vehicles
        for jVeh = 1:nVeh
            predicted_trims = repmat(trims_measured(jVeh), 1, Hp+1); % current trim and predicted trims in the prediction horizon

            x0 = x0_measured(jVeh,indices().x); % vehicle position x
            y0 = x0_measured(jVeh,indices().y); % vehicle position y

            predicted_lanelets = get_predicted_lanelets(scenario.vehicles(jVeh),predicted_trims(1),x0,y0,scenario.mpa,scenario.dt);

            predicted_occupied_areas = {}; % for initial time step, the occupied areas are not predicted yet
            scenario.vehicles(jVeh).communicate.send_message(scenario.k, predicted_trims, predicted_lanelets, predicted_occupied_areas);   
        end
    end
end

