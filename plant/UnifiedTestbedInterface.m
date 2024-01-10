classdef UnifiedTestbedInterface < Plant
    % UNIFIEDTESTBEDINTERFACE    Instance of experiment interface for usage via the unified testbed interface.

    properties (Access = private)
        comm_node %matlabParticipant
        subscription_experimentState
        subscription_plannerInvocation % = vehicleStateList
        client_testbedCharacteristics
        client_scaleRegistration
        client_mapDefinition % for defining the map
        client_mapRequest % for receiving the defined map
        publisher_readyState
        publisher_trajectoryCommand
        publisher_motionPlannerPeriod
        actionClient_vehiclesRequest
        goal_msg % Contains the action client goal message
        goal_handle % Handle for action client goal

        testbed_characteristics
        got_start = false
        got_stop = false

        dt_period_nanos
        sample
        out_of_map_limits
    end

    methods

        function obj = UnifiedTestbedInterface()
            obj = obj@Plant();
        end

        function setup(obj, options, vehicle_ids_controlled)

            arguments
                obj (1, 1) UnifiedTestbedInterface
                options (1, 1) Config
                vehicle_ids_controlled (1, :) uint8
            end

            % FIXME what does the UTI need? IDs, indices, all, controlled?

            obj.got_start = false;
            obj.got_stop = false;

            % create ros2 messages, nodes, publishers and subscribers
            obj.prepare_ros2();

            % middleware period for valid_after stamp
            obj.dt_period_nanos = uint64(options.dt_seconds * 1e9);

            % perform preparation phase (uses dt_period_nanos)
            obj.prepare_api(vehicle_ids_controlled);

            setup@Plant(obj, options);

            % receive sample for initial vehicle states
            sample_for_setup = obj.receive_new_sample();

            % take list of VehicleStates from sample
            state_list = sample_for_setup.vehicle_states;

            % since there is no steering info in [rad],
            % the initial_steering is assumed to 0
            initial_steering = 0;

            for index = 1:length(state_list)

                % cast state_list vehicle_id from int32 to uint8
                % to match the data type of superclass member
                % since this class is currently not supported,
                % it is accepted that higher vehicle ids cannot not be represented
                if ~any(uint8(state_list(index).vehicle_id) == vehicle_ids_controlled)
                    % if vehicle is not controlled, do not use received information
                    continue
                end

                % boolean with exactly one true entry for the position in all_vehicle_ids
                is_in_vehicle_list = uint8(state_list(index).vehicle_id) == all_vehicle_ids;

                obj.measurements(is_in_vehicle_list) = PlantMeasurement( ...
                    state_list(index).pose.x, ...
                    state_list(index).pose.y, ...
                    state_list(index).pose.yaw, ...
                    state_list(index).speed.linear, ...
                    initial_steering ...
                );

            end

        end

        function register_map(obj, map_as_string)
            % UTI allows to set a scenario map in the testbed.
            obj.set_map_in_testbed(map_as_string);
        end

        function map_as_string = receive_map(obj)
            % UTI allows to receive a default map from the testbed.
            % The return value is the file content of an .osm file as string.
            map_as_string = obj.set_map_in_testbed();
        end

        function synchronize_start_with_plant(obj)
            % Send ready signal for all assigned vehicle ids and the scenario
            ready_msg = ros2message(obj.publisher_readyState);
            ready_msg.entity = 'scenario';
            send(obj.publisher_readyState, ready_msg);

            for veh_id = obj.vehicle_ids_controlled
                ready_msg = ros2message(obj.publisher_readyState);
                ready_msg.entity = 'vehicle_controller';
                ready_msg.id = int32(veh_id);
                send(obj.publisher_readyState, ready_msg);
            end

            disp(strcat('Successfully sent ready messages for the scenario and the following vehicle ids: ', num2str(obj.vehicle_ids_controlled)));

            % Wait until the callback function on_experiment_state_change registered a start/stop signal
            while (~obj.got_stop && ~obj.got_start)
                pause(0.1);
            end

            disp('Start/Stop signal received. Leave setup.');
        end

        function [cav_measurements, hdv_measurements] = measure(obj)
            % take cav_measurements that were applied in the previous step
            % for the first step they hold the initialized values
            cav_measurements = obj.measurements;

            new_sample = obj.receive_new_sample();

            time_old_sample_predicted = uint64(obj.sample.current_time.sec) * 10^9 + uint64(obj.sample.current_time.nanosec) + uint64(obj.dt_period_nanos) * 1.5;
            time_new_sample = uint64(new_sample.current_time.sec) * 10^9 + uint64(new_sample.current_time.nanosec);

            if time_old_sample_predicted < time_new_sample
                warning(['The time of the received sample is bigger than expected. Missed deadline? ' ...
                         'Time old: %u sec, %u nanosec. Time received: %u sec, %u nanosec'], ...
                    obj.sample.current_time.sec, obj.sample.current_time.nanosec, ...
                    new_sample.current_time.sec, new_sample.current_time.nanosec);
            end

            obj.sample = new_sample;

            % if there are no manual vehicles return directly
            if ~obj.manual_control_config.is_active
                hdv_measurements = [];
                return
            end

            state_list = obj.sample.vehicle_states;

            % initialize return variable
            hdv_measurements(obj.manual_control_config.amount, 1) = PlantMeasurement();

            % since there is no steering info in [rad],
            % the hdv_steering is assumed to 0
            hdv_steering = 0;

            for index = 1:length(state_list)

                % cast state_list vehicle_id from int32 to uint8
                % to match the data type of the manual_control_config member
                % since this class is currently not supported,
                % it is accepted that higher vehicle ids cannot not be represented
                if ~any(uint8(state_list(index).vehicle_id) == obj.manual_control_config.hdv_ids)
                    % if vehicle is no hdv, do not use received information
                    continue
                end

                % boolean with exactly one true entry for the position in hdv_ids
                is_in_hdv_list = uint8(state_list(index).vehicle_id) == obj.manual_control_config.hdv_ids;

                hdv_measurements(is_in_hdv_list) = PlantMeasurement( ...
                    state_list(index).pose.x, ...
                    state_list(index).pose.y, ...
                    state_list(index).pose.yaw, ...
                    state_list(index).speed.linear, ...
                    hdv_steering ...
                );

            end

        end

        function apply(obj, info, ~, ~, mpa)
            y_pred = info.y_predicted;
            % simulate change of state
            for iVeh = obj.vehicle_indices_controlled
                obj.measurements(iVeh) = PlantMeasurement( ...
                    info.next_node(iVeh, NodeInfo.x), ...
                    info.next_node(iVeh, NodeInfo.y), ...
                    info.next_node(iVeh, NodeInfo.yaw), ...
                    mpa.trims(info.next_node(iVeh, NodeInfo.trim)).speed, ...
                    mpa.trims(info.next_node(iVeh, NodeInfo.trim)).steering ...
                );
            end

            % calculate vehicle control messages
            obj.out_of_map_limits = false(obj.amount, 1);

            for iVeh = obj.vehicle_indices_controlled
                n_traj_pts = obj.Hp;
                n_predicted_points = size(y_pred{iVeh}, 1);
                idx_predicted_points = 1:n_predicted_points / n_traj_pts:n_predicted_points;
                trajectory_points(1:n_traj_pts) = ros2message('uti_msgs/TrajectoryPoint');

                for i_traj_pt = 1:n_traj_pts
                    i_predicted_points = idx_predicted_points(i_traj_pt);
                    trajectory_points(i_traj_pt).t = obj.enhance_timepoint(obj.sample.current_time, i_traj_pt * obj.dt_period_nanos);
                    trajectory_points(i_traj_pt).px = y_pred{iVeh}(i_predicted_points, 1);
                    trajectory_points(i_traj_pt).py = y_pred{iVeh}(i_predicted_points, 2);

                    yaw = y_pred{iVeh}(i_predicted_points, 3);

                    speed = mpa.trims(y_pred{iVeh}(i_predicted_points, 4)).speed;

                    trajectory_points(i_traj_pt).vx = cos(yaw) * speed;
                    trajectory_points(i_traj_pt).vy = sin(yaw) * speed;
                end

                obj.out_of_map_limits(iVeh) = obj.is_veh_at_map_border(trajectory_points);

                vehicle_command_trajectory = ros2message(obj.publisher_trajectoryCommand);
                vehicle_command_trajectory.vehicle_id = int32(obj.all_vehicle_ids(iVeh));
                vehicle_command_trajectory.trajectory = trajectory_points;
                vehicle_command_trajectory.t_creation = obj.enhance_timepoint(obj.sample.current_time, 0); % Just use as conversion function
                vehicle_command_trajectory.t_valid_after = obj.enhance_timepoint(obj.sample.current_time, obj.dt_period_nanos + 1);

                send(obj.publisher_trajectoryCommand, vehicle_command_trajectory);
            end

        end

        function got_stop = is_stop(obj)
            got_stop = false;

            if any(obj.out_of_map_limits) || obj.got_stop
                got_stop = true;
            end

        end

        function end_run(obj)
            disp('End')
        end

    end

    methods (Access = private)

        function on_experiment_state_change(obj, msg)
            % Message msg is of type std_msgs/String
            disp(strcat('Experiment state change: ', msg.data));

            if strcmp(msg.data, 'start')
                obj.got_start = true;
            elseif strcmp(msg.data, 'stop')

                if (obj.got_start)
                    % Only set got_stop=true if experiment has been started before
                    obj.got_stop = true;
                end

            else
                warning(strcat('Received an unknown experiment state: ', msg.data));
            end

        end

        % helper function
        function stop_experiment = is_veh_at_map_border(obj, trajectory_points)
            % Vehicle command timeout is 1000 ms after the last valid_after_stamp,
            % so vehicle initiates stop between third and fourth trajectory point
            % vhlength = 0.25;
            vhwidth = 0.1;
            x_min = vhwidth / 2 + 0; % vhlength + 0;
            x_max = -vhwidth / 2 + 4.5; % -vhlength + 4.5;
            y_min = vhwidth / 2 + 0; % vhlength + 0;
            y_max = -vhwidth / 2 + 4.0; % -vhlength + 4.0;
            px = trajectory_points(4).px;
            py = trajectory_points(4).py;
            stop_experiment = x_min > px || px > x_max ...
                || y_min > py || py > y_max;
        end

        function timepoint = enhance_timepoint(obj, timepoint, dt_nanos)
            % timepoint: a time stamp in format as defined in builtin_interfaces/Time (ROS2)
            % dt_nanos: positive delta in nanoseconds by which the timepoint shall be enhanced
            % return: new time point in format as defined in builtin_interfaces/Time (ROS2)

            % Since Matlab 2022a doesn't have ros2time yet, this function
            % defines the conversion
            t_new_sec = timepoint.sec + int32(idivide((uint64(timepoint.nanosec) + uint64(dt_nanos)), uint64(1000000000), 'floor'));
            t_new_nanosec = uint32(mod(uint64(timepoint.nanosec) + uint64(dt_nanos), 1000000000));
            timepoint.sec = t_new_sec;
            timepoint.nanosec = t_new_nanosec;
        end

        function map_as_string = set_map_in_testbed(obj, map_as_string)
            % map_as_string: must be a lanelet2 map given as string of .osm
            % file. If empty, the default map of the testbed is requested.
            % return: Either the given map or the received map.

            % Request map to use
            map_definition_request = ros2message(obj.client_mapDefinition);

            if nargin == 1 % no map given--> request testbed's default map
                map_definition_request.use_default = true;
                map_definition_request.map = '';
            else
                map_definition_request.use_default = false;
                map_definition_request.map = map_as_string;
            end

            map_definition_response = call(obj.client_mapDefinition, map_definition_request);

            if (~map_definition_response.valid)
                error('Map definition was not successful. Scaling service returned an error.');
            end

            if (~map_definition_response.ok)
                error(strcat('The testbed rejected the requested map. Error message: ', map_definition_response.msg));
            end

            disp('Successfully registered map.');

            if nargin == 1 % no map given--> testbed's default was successfully requested but now we still want to get it
                map_request = ros2message(obj.client_mapRequest);
                map_request_response = call(obj.client_mapRequest, map_request);

                if (~map_request_response.defined)
                    error('Map request was not successful. The testbed returned that no map is currently defined.');
                end

                if (~map_request_response.valid)
                    error('Map request was not successful. Scaling service returned an error.');
                end

                map_as_string = map_request_response.map;
            end

        end

        function prepare_ros2(obj)

            disp('Setup. Phase 1: Generation of ROS2 messages...');

            % generated message types if not already existing
            uti_ros2gen();

            disp('Setup. Phase 2: Creation of all reader and writes...');
            %matlabDomainId = 1; %TODO: If not working try str2double(getenv('DDS_DOMAIN'))
            % create ros node for communication
            obj.comm_node = ros2node("matlab_pdmpc_ros_node");

            % listen for changes of experiment state (start and stop of experiment)
            obj.subscription_experimentState = ros2subscriber(obj.comm_node, "/experiment_state", "std_msgs/String", ...
                @obj.on_experiment_state_change, Durability = "transientlocal", Reliability = "reliable", ...
                History = "keeplast", Depth = 2);

            % create subscription for planner invocation without a callback since we want to activly wait
            % only keep the last message in the queue, i.e., we throw away missed ones
            obj.subscription_plannerInvocation = ros2subscriber(obj.comm_node, "/planner_invocation", "uti_msgs/VehicleStateList", "History", "keeplast", "Depth", 1);

            % create client such that we can ask for the testbed characteristics in the preparation phase
            obj.client_testbedCharacteristics = ros2svcclient(obj.comm_node, '/testbed_characteristics_request', 'uti_msgs/TestbedCharacteristics');

            % create client such that we can register the scale we want to use within the scaling node
            obj.client_scaleRegistration = ros2svcclient(obj.comm_node, '/scale_registration', 'uti_msgs/ScaleRegistration');

            % create client with which we can define the map we want to use
            obj.client_mapDefinition = ros2svcclient(obj.comm_node, '/map_definition_request', 'uti_msgs/MapDefinition');

            % create client with which we can receive the defined map
            obj.client_mapRequest = ros2svcclient(obj.comm_node, '/map_request', 'uti_msgs/MapRequest');

            % create publisher for ready state
            obj.publisher_readyState = ros2publisher(obj.comm_node, '/ready_state', 'uti_msgs/Ready');

            % create publisher for trajectory commands
            obj.publisher_trajectoryCommand = ros2publisher(obj.comm_node, '/trajectory_command', 'uti_msgs/TrajectoryCommand');

            % create publisher for motion planner period
            obj.publisher_motionPlannerPeriod = ros2publisher(obj.comm_node, '/motion_planner_period', 'builtin_interfaces/Duration');

            % create client with which we can ask the testbed for specific vehicles
            [obj.actionClient_vehiclesRequest, obj.goal_msg] = ros2actionclient(obj.comm_node, '/vehicles_request', 'uti_msgs/VehiclesRequest');
            % Since matlab does not provide support for actions, we use a normal message via the action bridge node
            % obj.actionClient_vehiclesRequest = ros2publisher(obj.comm_node, '/vehicles_request_action_bridge_goal', 'uti_msgs/VehicleIDs');
        end

        function prepare_api(obj, vehicle_ids_controlled)
            disp('Setup. Phase 3: Perform preparation phase...');

            % Request scaling of 1:18
            disp('Wait for testbed nodes to become available...');
            [connectionStatus, connectionStatustext] = waitForServer(obj.client_scaleRegistration);

            if (~connectionStatus)
                error(strcat('Scaling service could not be reached. Status text: ', connectionStatustext));
            end

            disp('Scaling node available. Assume all other nodes to be available as well...');
            scaling_request = ros2message(obj.client_scaleRegistration);
            scaling_request.entity = 'user';
            scaling_request.scale = uint16(18);
            scaling_response = call(obj.client_scaleRegistration, scaling_request);

            if (~scaling_response.ok)
                error('Registration of scaling was not successful.');
            end

            disp(strcat('Successfully registered scaling of 1:', num2str(scaling_request.scale)));

            % Request testbed characteristics
            testbed_characteristics_request = ros2message(obj.client_testbedCharacteristics);
            obj.testbed_characteristics = call(obj.client_testbedCharacteristics, testbed_characteristics_request);

            if (~obj.testbed_characteristics.valid)
                error('testbed characteristics request was not successful. Scaling service returned an error.');
            end

            disp('Successfully received testbed characteristics.');

            % Set motion planner period
            motion_planner_period_request = ros2message(obj.publisher_motionPlannerPeriod);
            motion_planner_period_request.nanosec = uint32(obj.dt_period_nanos);
            send(obj.publisher_motionPlannerPeriod, motion_planner_period_request);

            disp('Sent request for motion planner period.');

            % Request the vehicle ids which shall be used in this experiment (this should be an action, but we use only the initial message)
            % TODO: This assumes that the assigned vehicles are all vehicles needed in the experiment. Correct?
            obj.goal_msg.vehicle_ids = int32(vehicle_ids_controlled);
            callbackOpts = ros2ActionSendGoalOptions(FeedbackFcn = @obj.vehicleRequestActionFeedbackCallback, ResultFcn = @obj.vehicleRequestActionResultCallback); % Currently, we don't use it...
            [connectionStatus, connectionStatustext] = waitForServer(obj.actionClient_vehiclesRequest);

            if (~connectionStatus)
                error(strcat('Action server could not be reached. Status text: ', connectionStatustext));
            end

            obj.goal_handle = sendGoal(obj.actionClient_vehiclesRequest, obj.goal_msg, callbackOpts);
            disp('Sent message to define the vehicle ids. We assume that goal was accepted, so no further test...');
        end

        function new_sample = receive_new_sample(obj)
            % Receive new messages. In order to recognize stop signals, we
            % have a timeout of 2 seconds and then check again, whether a
            % stop signal was received...
            new_sample_received = false;

            while (~new_sample_received)

                try
                    new_sample = receive(obj.subscription_controllerInvocation, 1);
                    new_sample_received = true;
                catch % Timeout

                    if (obj.got_stop)
                        disp('Stop signal received. Stop waiting for new messages.');
                        % Set new_sample to the one of the last timestep
                        % such that we can leave this function
                        % successfully. Afterwards the calling function
                        % will detect that obj.got_stop=true.
                        new_sample = obj.sample;
                        new_sample_received = true;
                    end

                end

            end

        end

        function vehicleRequestActionFeedbackCallback(obj, goalHandle, feedbackMsg)
            disp('Received feedback on the action goal, but nothing we do here with it...');
        end

        function vehicleRequestActionResultCallback(obj, goalHandle, wrappedResultMsg)
            disp('Received result on the action goal, but nothing we do here with it...');
        end

    end

end
