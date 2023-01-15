function create_subscribers(hlc)
% create_subscribers This function creates the ROS subscribers for the given hlc.
% ROS 2 nodes are created for each vehicle. Each vehicle has its own topic
% and sends its data only to its own topic.
%
% INPUT:
%   hlc: Controller implementing the HLCInterface
%

% init global message storage for callbacks
% initialize the global variable
global stored_traffic_msgs;
stored_traffic_msgs = struct('time_step',[],'vehicle_id',[],'is_fallback',[],'current_pose',[],'current_trim_index',[],'predicted_lanelets',[],'occupied_areas',[],'reachable_sets',[], 'MessageType',[]);

% Create subscribers.
% Each vehicle subscribes all other vehicles.
% NOTE that subscribers are create only once but not loopover all
% vehicles to let all of them subscribe others because it is
% time-consuming to create many subscribers.
% The subscribers will be used by the given HLC.
disp('Creating ROS 2 subscribers...')
start = tic;
if length(hlc.indices_in_vehicle_list) ~= 1
    % HLC responsible for all vehicle
    % subscribeto all vehicles. HLC will read from its own publishers to
    % simulate distributed behaviour
    veh_indices_to_be_subscribed = 1:hlc.scenario.options.amount;
else
    % HLC responsible for 1 vehicle
    % subscribeto only to other vehicles. Given HLC will read from other HLCs
    veh_indices_to_be_subscribed = setdiff(1:hlc.scenario.options.amount, hlc.indices_in_vehicle_list);
end
veh_ids_to_be_subscribed = [hlc.scenario.options.veh_ids(veh_indices_to_be_subscribed)];
amount = hlc.scenario.options.amount;
% subscribe to all other vehicles
hlc.ros_subscribers.traffic = create_subscriber(hlc.scenario.vehicles(hlc.indices_in_vehicle_list(1)).communicate.traffic,veh_indices_to_be_subscribed, veh_ids_to_be_subscribed, amount);
% subscribe to all vehicles
hlc.ros_subscribers.predictions = create_subscriber(hlc.scenario.vehicles(hlc.indices_in_vehicle_list(1)).communicate.predictions,1:hlc.scenario.options.amount, hlc.scenario.options.veh_ids, amount);
if length(hlc.indices_in_vehicle_list) == 1
    pause(5.0) % wait for all subscribers to be created in distributed case, because otherwise early sent messages will be lost.
end
duration = toc(start);
disp(['Finished creating subscribers in ' num2str(duration) ' seconds.'])
end

