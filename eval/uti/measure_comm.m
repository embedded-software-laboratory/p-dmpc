disp('Setup. Phase 1: Generation of ROS2 messages...');

veh_ids = [1,2,3];
n_veh = 3;

% generated message types if not already existing
uti_ros2gen();

disp('Setup. Phase 2: Creation of all reader and writes...');
%matlabDomainId = 1; %TODO: If not working try str2double(getenv('DDS_DOMAIN'))
% create ros node for communication
comm_node = ros2node("matlab_pdmpc_ros_node");

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% listen for changes of experiment state (start and stop of experiment)
subscription_experimentState = ros2subscriber(comm_node, "/experiment_state", "std_msgs/String", ...
    @on_experiment_state_change, Durability = "transientlocal", Reliability = "reliable", ...
    History = "keeplast", Depth = 2);

% create subscription for controller invocation without a callback since we want to activly wait
% only keep the last message in the queue, i.e., we throw away missed ones
subscription_plannerInvocation = ros2subscriber(comm_node, "/planner_invocation", "uti_msgs/VehicleStateList", "History", "keeplast", "Depth", 1);

% create client such that we can ask for the testbed characteristics in the preparation phase
client_testbedCharacteristics = ros2svcclient(comm_node, '/testbed_characteristics_request', 'uti_msgs/TestbedCharacteristics');

% create client such that we can register the scale we want to use within the scaling node
client_scaleRegistration = ros2svcclient(comm_node, '/scale_registration', 'uti_msgs/ScaleRegistration');

% create client with which we can define the map we want to use
client_mapDefinition = ros2svcclient(comm_node, '/map_definition_request', 'uti_msgs/MapDefinition');

% create client with which we can receive the defined map
client_mapRequest = ros2svcclient(comm_node, '/map_request', 'uti_msgs/MapRequest');

% create publisher for ready state
publisher_readyState = ros2publisher(comm_node, '/ready_state', 'uti_msgs/Ready');

% create publisher for trajectory commands
publisher_trajectoryCommand = ros2publisher(comm_node, '/trajectory_command', 'uti_msgs/TrajectoryCommand');

% create publisher for motion planner period
publisher_motionPlannerPeriod = ros2publisher(comm_node, '/motion_planner_period', 'builtin_interfaces/Duration');

% create client with which we can ask the lab for specific vehicles
[actionClient_vehiclesRequest, goal_msg] = ros2actionclient(comm_node, '/vehicles_request', 'uti_msgs/VehiclesRequest');
% Since matlab does not provide support for actions, we use a normal message via the action bridge node
% actionClient_vehiclesRequest = ros2publisher(comm_node, '/vehicles_request_action_bridge_goal', 'uti_msgs/VehicleIDs');

% % NEEDED???
% % create writer for lab visualization
% matlabVisualizationTopicName = 'visualization';
% writer_visualization = DDS.DataWriter(DDS.Publisher(matlabParticipantLab), 'Visualization', matlabVisualizationTopicName);

% Middleware period for valid_after stamp
dt_period_nanos = uint64(0.2 * 1e9);

disp('Setup. Phase 3: Perform preparation phase...');

% if (~map_comm_done) % Map was not already defined in lab
%     set_map_in_lab(fileread(scenario.road_data_file_path));
% end

% Request scaling of 1:18
disp('Wait for lab nodes to become available...');
[connectionStatus, connectionStatustext] = waitForServer(client_scaleRegistration);

if (~connectionStatus)
    error(strcat('Scaling service could not be reached. Status text: ', connectionStatustext));
end

pause(0.5);

disp('Scaling node available. Assume all other nodes to be available as well...');
scaling_request = ros2message(client_scaleRegistration);
scaling_request.entity = 'user';
scaling_request.scale = uint16(18);
scaling_response = call(client_scaleRegistration, scaling_request);

if (~scaling_response.ok)
    error('Registration of scaling was not successful.');
end

disp(strcat('Successfully registered scaling of 1:', num2str(scaling_request.scale)));

% Request map to use
map_definition_request = ros2message(client_mapDefinition);

map_definition_request.use_default = true;
map_definition_request.map = '';
% else
%     map_definition_request.use_default = false;
%     map_definition_request.map = map_as_string;
% end

map_definition_response = call(client_mapDefinition, map_definition_request);

if (~map_definition_response.valid)
    error('Map definition was not successful. Scaling service returned an error.');
end

if (~map_definition_response.ok)
    error(strcat('The lab rejected the requested map. Error message: ', map_definition_response.msg));
end

disp('Successfully registered map.');

% if nargin == 1 % no map given--> lab's default was successfully requested but now we still want to get it
%     map_request = ros2message(client_mapRequest);
%     map_request_response = call(client_mapRequest, map_request);
%
%     if (~map_request_response.defined)
%         error('Map request was not successful. The lab returned that no map is currently defined.');
%     end
%
%     if (~map_request_response.valid)
%         error('Map request was not successful. Scaling service returned an error.');
%     end
%
%     map_as_string = map_request_response.map;
% end


% Request testbed characteristics
testbed_characteristics_request = ros2message(client_testbedCharacteristics);
testbed_characteristics = call(client_testbedCharacteristics, testbed_characteristics_request);

if (~testbed_characteristics.valid)
    error('testbed characteristics request was not successful. Scaling service returned an error.');
end

disp('Successfully received testbed characteristics.');

% Set motion planner period
motion_planner_period_request = ros2message(publisher_motionPlannerPeriod);
motion_planner_period_request.nanosec = uint32(dt_period_nanos);
send(publisher_motionPlannerPeriod, motion_planner_period_request);

disp('Sent request for motion planner period.');

% Request the vehicle ids which shall be used in this experiment (this should be an action, but we use only the initial message)
% TODO: This assumes that the assigned vehicles are all vehicles needed in the experiment. Correct?
goal_msg.vehicle_ids = int32(veh_ids);
callbackOpts = ros2ActionSendGoalOptions(FeedbackFcn = @vehicleRequestActionFeedbackCallback, ResultFcn = @vehicleRequestActionResultCallback); % Currently, we don't use it...
[connectionStatus, connectionStatustext] = waitForServer(actionClient_vehiclesRequest);

if (~connectionStatus)
    error(strcat('Action server could not be reached. Status text: ', connectionStatustext));
end

goal_handle = sendGoal(actionClient_vehiclesRequest, goal_msg, callbackOpts);
disp('Sent message to define the vehicle ids. We assume that goal was accepted, so no further test...');

% Send ready signal for all assigned vehicle ids and the scenario
ready_msg = ros2message(publisher_readyState);
ready_msg.entity = 'scenario';
send(publisher_readyState, ready_msg);

for veh_id = veh_ids
    ready_msg = ros2message(publisher_readyState);
    ready_msg.entity = 'motion_planner';
    ready_msg.id = int32(veh_id);
    send(publisher_readyState, ready_msg);
end

disp(strcat('Successfully sent ready messages for the scenario and the following vehicle ids: ', num2str(veh_ids)));


disp('Start loop...');


% Generate trajectories upfront
n_traj_pts = 4;
trajectory_points(1:n_traj_pts) = ros2message('uti_msgs/TrajectoryPoint');
for i_traj_pt = 1:n_traj_pts
    % Only dummy values
    trajectory_points(i_traj_pt).t = struct('sec',int32(0),'nanosec',uint32(0));
    trajectory_points(i_traj_pt).px = 0;
    trajectory_points(i_traj_pt).py = 0;
    trajectory_points(i_traj_pt).vx = 0;
    trajectory_points(i_traj_pt).vy = 0;
end

vehicle_command_trajectory(1:n_veh) = ros2message(publisher_trajectoryCommand);
for iVeh = veh_ids
    vehicle_command_trajectory(iVeh) = ros2message(publisher_trajectoryCommand);
    vehicle_command_trajectory(iVeh).vehicle_id = int32(veh_ids(iVeh));
    vehicle_command_trajectory(iVeh).trajectory = trajectory_points;
    vehicle_command_trajectory(iVeh).t_creation = struct('sec',int32(0),'nanosec',uint32(0));
    vehicle_command_trajectory(iVeh).t_valid_after = struct('sec',int32(0),'nanosec',uint32(0));
    %ros2time(comm_node,"now");
end


while (true)
    new_sample_received = false;
    while (~new_sample_received)
        try
            new_sample = receive(subscription_plannerInvocation, 1);
            %disp(new_sample.eval-ros2time(comm_node,"now"));
            new_sample_received = true;
        catch % Timeout
        end
    end

    sample = new_sample;


    for iVeh = veh_ids
        vehicle_command_trajectory(iVeh).eval = sample.eval; % Just forward time. Subtraject computation time of P-DMPC here later on
        send(publisher_trajectoryCommand, vehicle_command_trajectory(iVeh));
    end
end






function vehicleRequestActionFeedbackCallback(goalHandle, feedbackMsg)
    disp('Received feedback on the action goal, but nothing we do here with it...');
end

function vehicleRequestActionResultCallback(goalHandle, wrappedResultMsg)
    disp('Received result on the action goal, but nothing we do here with it...');
end

function on_experiment_state_change(msg)
    % Message msg is of type std_msgs/String
    disp(strcat('Experiment state change: ', msg.data));
end