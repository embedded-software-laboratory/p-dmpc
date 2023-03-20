classdef UnifiedLabAPI < InterfaceExperiment
% CPMLAB    Instance of experiment interface for usage via the generalized lab api.
    
    properties (Access=private)
        comm_node %matlabParticipant
        % matlabParticipantLab
        subscription_experimentState
        subscription_controllerInvocation % = vehicleStateList
        client_labProperties
        client_scaleRegistration
        client_mapDefinition
        publisher_readyState
        publisher_trajectoryCommand
        actionClient_vehiclesRequest

        experiment_state = "preparation"
        lab_properties
        got_start = false
        got_stop = false
        writer_visualization % WHY?
        
        dt_period_nanos
        sample
        out_of_map_limits
        pos_init
    end
    
    methods
        function obj = UnifiedLabAPI(scenario, veh_ids)
            obj = obj@InterfaceExperiment(scenario, veh_ids);
            obj.pos_init = false;
            obj.cur_node = node(0, [obj.scenario.vehicles(:).trim_config], [obj.scenario.vehicles(:).x_start]', [obj.scenario.vehicles(:).y_start]', [obj.scenario.vehicles(:).yaw_start]', zeros(obj.amount,1), zeros(obj.amount,1));
            if ispc
                error('You are using a Windows machine, please do not select lab mode!')
            end
        end
        
        function setup(obj)
            assert(issorted(obj.veh_ids));

            obj.got_start = false;
            obj.got_start = false;

            % Initialize data readers/writers...
            % % getenv('HOME'), 'dev/software/high_level_controller/examples/matlab' ...
            % common_cpm_functions_path = fullfile( ...
            %     getenv('HOME'), 'dev/software/high_level_controller/examples/matlab' ...
            % );
            % assert(isfolder(common_cpm_functions_path), 'Missing folder "%s".', common_cpm_functions_path);
            % addpath(common_cpm_functions_path);

            disp('Setup. Phase 1: Generation of ROS2 messages...');

            % generated message types if not already existing
            ula_ros2gen();

            disp('Setup. Phase 2: Creation of all reader and writes...');
            matlabDomainId = 1; %TODO: If not working try str2double(getenv('DDS_DOMAIN'))
            % create ros node for communication
            obj.comm_node = ros2node("matlab_pdmpc_ros_node");

            % listen for changes of experiment state (start and stop of experiment)
            obj.subscription_experimentState = ros2subscriber(obj.comm_node, "/experiment_state", "std_msgs/String", @obj.on_experiment_state_change);

            % create subscription for controller invocation without a callback since we want to activly wait
            % only keep the last message in the queue, i.e., we throw away missed ones
            obj.subscription_controllerInvocation = ros2subscriber(obj.comm_node, "/controller_invocation", "ula_interfaces/VehicleStateList","History","keeplast","Depth",1);

            % create client such that we can ask for the lab properties in the preparation phase
            obj.client_labProperties = ros2svcclient(obj.comm_node, '/lab_properties_request', 'ula_interfaces/LabProperties');

            % create client such that we can register the scale we want to use within the scaling node
            obj.client_scaleRegistration = ros2svcclient(obj.comm_node, '/scale_registration', 'ula_interfaces/ScaleRegistration');

            % create client with which we can define the map we want to use
            obj.client_mapDefinition = ros2svcclient(obj.comm_node, '/map_definition_request', 'ula_interfaces/MapDefinition');

            % create publisher for ready state
            obj.publisher_readyState = ros2publisher(obj.comm_node, '/ready_state', 'ula_interfaces/Ready');

            % create publisher for trajectory commands
            obj.publisher_trajectoryCommand = ros2publisher(obj.comm_node, '/trajectory_command', 'ula_interfaces/TrajectoryCommand');
            
            % create client with which we can ask the lab for specific vehicles
            % Since matlab does not provide support for actions, we use a normal message via the action bridge node
            obj.actionClient_vehiclesRequest = ros2publisher(obj.comm_node, '/vehicles_request_action_bridge_goal', 'ula_interfaces/VehicleIDs');

            % % NEEDED???
            % % create writer for lab visualization
            % matlabVisualizationTopicName = 'visualization';
            % obj.writer_visualization = DDS.DataWriter(DDS.Publisher(obj.matlabParticipantLab), 'Visualization', matlabVisualizationTopicName);

            % Middleware period for valid_after stamp
            obj.dt_period_nanos = uint64(obj.scenario.options.dt*1e9);


            disp('Setup. Phase 3: Perform preparation phase...');

            % Request scaling of 1:18
            disp('Wait for lab nodes to become available...');
            [connectionStatus,connectionStatustext] = waitForServer(obj.client_scaleRegistration);
            if (~connectionStatus)
                error(strcat('Scaling service could not be reached. Status text: ', connectionStatustext));
            end
            disp('Scaling node available. Assume all other nodes to be available as well...');
            scaling_request = ros2message(obj.client_scaleRegistration);
            scaling_request.entity = 'user';
            scaling_request.scale = uint16(18);
            scaling_response = call(obj.client_scaleRegistration,scaling_request);
            if (~scaling_response.ok)
                error('Registration of scaling was not successful.');
            end
            disp(strcat('Successfully registered scaling of 1:', num2str(scaling_request.scale)));

            % Request map to use
            map_definition_request = ros2message(obj.client_mapDefinition);
            map_definition_request.use_default = true; % TODO
            map_definition_request.map = '';
            map_definition_response = call(obj.client_mapDefinition,map_definition_request);
            if (~map_definition_response.valid)
                error('Map definition was not successful. Scaling service returned an error.');
            end
            if (~map_definition_response.ok)
                error(strcat('The lab rejected the requested map. Error message: ', map_definition_response.msg));
            end
            disp('Successfully registered map.');

            % Request lab properties
            lab_properties_request = ros2message(obj.client_labProperties);
            obj.lab_properties = call(obj.client_labProperties,lab_properties_request);
            if (~obj.lab_properties.valid)
                error('Lab properties request was not successful. Scaling service returned an error.');
            end
            disp('Successfully registered lab properties.');

            % Request the vehicle ids which shall be used in this experiment (this should be an action, but we use only the initial message)
            % TODO: This assumes that the assigned vehicles are all vehicles needed in the experiment. Correct?
            vehicles_msg = ros2message(obj.actionClient_vehiclesRequest);
            assignin('base','vehicles',int32(obj.veh_ids));
            vehicles_msg.vehicle_ids = int32(obj.veh_ids);
            send(obj.actionClient_vehiclesRequest, vehicles_msg);
            warning('Send message to define the vehicle ids. Since matlab does not support actions currently, we do not react on any response...');

            % TODO: Set vehicle_controller_period

            % Send ready signal for all assigned vehicle ids and the scenario
            ready_msg = ros2message(obj.publisher_readyState);
            ready_msg.entity = 'scenario';
            send(obj.publisher_readyState, ready_msg);
            for veh_id = obj.veh_ids
                ready_msg = ros2message(obj.publisher_readyState);
                ready_msg.entity = 'vehicle_controller';
                ready_msg.id = int32(veh_id);
                send(obj.publisher_readyState, ready_msg);
            end
            disp(strcat('Successfully sent ready messages for the scenario and the following vehicle ids: ', num2str(obj.veh_ids)));
            
            % Wait until the callback function on_experiment_state_change registered a start/stop signal
            while (~obj.got_stop && ~obj.got_start)
                pause(0.1);
            end
            disp('Start/Stop signal received. Leave setup.');
        end

        function [ x0, trim_indices ] = measure(obj)
            disp('Measure');
            new_sample = receive(obj.subscription_controllerInvocation);
            % Don't do the check in the first step (see later in this function for setting of pos_init)
            if obj.pos_init && ...
                    uint64(obj.sample.current_time.sec) * 10^9 + uint64(obj.sample.current_time.nanosec) + uint64(obj.dt_period_nanos) * 1.5 ...
                    < uint64(new_sample.current_time.sec) * 10^9 + uint64(new_sample.current_time.nanosec)
                warning(['The time of the received sample is bigger than expected. Missed deadline? ' ...
                    'Time old: %u sec, %u nanosec. Time received: %u sec, %u nanosec'], ...
                    obj.sample.current_time.sec, obj.sample.current_time.nanosec, ...
                    new_sample.current_time.sec, new_sample.current_time.nanosec);
            end
            obj.sample = new_sample;
            state_list = obj.sample.vehicle_states;

            x0 = zeros(obj.scenario.options.amount+obj.scenario.options.manual_control_config.amount,4);
            
            % for first iteration use real poses
            if (obj.pos_init == false)
                for index = 1:length(state_list)
                    if ismember(double(state_list(index).vehicle_id), obj.veh_ids) % measure cav states
                        list_index = obj.indices_in_vehicle_list(index); % use list to prevent breaking distributed control
                        x0(list_index,1) = state_list(index).pose.x;
                        x0(list_index,2) = state_list(index).pose.y;
                        x0(list_index,3) = state_list(index).pose.theta;
                        x0(list_index,4) = [state_list(index).speed.linear];
                    end
                end

                [ ~, trim_indices ] = obj.measure_node();
                obj.pos_init = true;
            else
                [ x0(1:obj.scenario.options.amount,:), trim_indices ] = obj.measure_node(); % get cav states from current node
            end

            % Always measure HDV
            hdv_index = 1;
            for index = 1:length(state_list)
                if ismember(double(state_list(index).vehicle_id), obj.scenario.options.manual_control_config.hdv_ids)
                    list_index = obj.scenario.options.amount+hdv_index; 
                    hdv_index = hdv_index + 1;
                    x0(list_index,1) = state_list(index).pose.x;
                    x0(list_index,2) = state_list(index).pose.y;
                    x0(list_index,3) = state_list(index).pose.theta;
                    x0(list_index,4) = [state_list(index).speed.linear];
                end
            end
        end
        
        function apply(obj, info, ~, k, scenario)
            y_pred = info.y_predicted;
            % simulate change of state
            for iVeh = obj.indices_in_vehicle_list
                obj.cur_node(iVeh,:) = info.next_node(iVeh,:);
            end
            obj.k = k;
            % calculate vehicle control messages
            obj.out_of_map_limits = false(obj.scenario.options.amount,1);
            for iVeh = obj.indices_in_vehicle_list
                n_traj_pts = obj.scenario.options.Hp;
                n_predicted_points = size(y_pred{iVeh},1);
                idx_predicted_points = 1:n_predicted_points/n_traj_pts:n_predicted_points;
                trajectory_points(1:n_traj_pts) = ros2message('ula_interfaces/TrajectoryPoint');
                for i_traj_pt = 1:n_traj_pts
                    i_predicted_points = idx_predicted_points(i_traj_pt);
                    trajectory_points(i_traj_pt).t = obj.enhance_timepoint(obj.sample.current_time, i_traj_pt*obj.dt_period_nanos);
                    trajectory_points(i_traj_pt).px = y_pred{iVeh}(i_predicted_points,1);
                    trajectory_points(i_traj_pt).py = y_pred{iVeh}(i_predicted_points,2);
                
                    yaw = y_pred{iVeh}(i_predicted_points,3);
                    
                    speed = scenario.mpa.trims(y_pred{iVeh}(i_predicted_points,4)).speed;
                    
                    trajectory_points(i_traj_pt).vx = cos(yaw)*speed;
                    trajectory_points(i_traj_pt).vy = sin(yaw)*speed;
                end

                obj.out_of_map_limits(iVeh) = obj.is_veh_at_map_border(trajectory_points);

                vehicle_command_trajectory = ros2message(obj.publisher_trajectoryCommand);
                vehicle_command_trajectory.vehicle_id = int32(obj.scenario.options.veh_ids(iVeh));
                vehicle_command_trajectory.trajectory = trajectory_points;
                vehicle_command_trajectory.t_creation = obj.enhance_timepoint(obj.sample.current_time, 0); % Just use as conversion function
                vehicle_command_trajectory.t_valid_after = obj.enhance_timepoint(obj.sample.current_time, obj.dt_period_nanos + 1);

                send(obj.publisher_trajectoryCommand, vehicle_command_trajectory);
            end
        end

        function visualize(obj, visualization_command)
            %obj.writer_visualization.write(visualization_command);
            disp('Visualize called, but what to do?');
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
    
    methods (Access=private)   
        function on_experiment_state_change(obj, msg)
            % Message msg is of type std_msgs/String
            if msg.data == 'start'
                obj.got_start = true;
            elseif msg.data == 'stop'
                obj.got_stop = true;
            else
                warning(strcat('Received an unknown experiment state: ', msg.data));
            end
            disp(strcat('Experiment state change: ',msg.data));
        end

        % helper function
        function stop_experiment = is_veh_at_map_border(obj, trajectory_points)
            % Vehicle command timeout is 1000 ms after the last valid_after_stamp,
            % so vehicle initiates stop between third and fourth trajectory point
            % vhlength = 0.25;
            vhwidth = 0.1;
            x_min =  vhwidth/2 + 0;  % vhlength + 0;
            x_max = -vhwidth/2 +  4.5; % -vhlength + 4.5;
            y_min =  vhwidth/2 + 0;  % vhlength + 0;
            y_max = -vhwidth/2 + 4.0; % -vhlength + 4.0;
            px = trajectory_points(4).px;
            py = trajectory_points(4).py;
            stop_experiment = x_min>px || px>x_max ...
                           || y_min>py || py>y_max;
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
    end
end
