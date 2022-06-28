classdef CPMLab < InterfaceExperiment
% CPMLAB    Instance of experiment interface for usage in the cpm lab.
    
    properties (Access=private)
        vehicle_ids
        matlabParticipant
        matlabParticipantLab
        reader_vehicleStateList
        writer_vehicleCommandTrajectory
        writer_vehicleCommandDirect
        writer_visualization
        reader_systemTrigger
        writer_readyStatus
        trigger_stop
        dt_period_nanos
        sample
        out_of_map_limits
        wheelNode
        wheelSub
        gamepadNode
        gamepadSub
        visualize_manual_lane_change_counter
        visualize_second_manual_lane_change_counter
        lastSteeringValue
        stored_wheel_msgs
        stored_gamepad_msgs
        g29_handler
        g29_last_position
    end
    
    methods
        function obj = CPMLab(scenario, vehicle_ids)
            obj.vehicle_ids = vehicle_ids;
            obj.scenario = scenario;
            obj.visualize_manual_lane_change_counter = 0;
            obj.visualize_second_manual_lane_change_counter = 0;
            obj.cur_node = node(0, [obj.scenario.vehicles(:).trim_config], [obj.scenario.vehicles(:).x_start]', [obj.scenario.vehicles(:).y_start]', [obj.scenario.vehicles(:).yaw_start]', zeros(obj.scenario.nVeh,1), zeros(obj.scenario.nVeh,1));
        end
        
        function setup(obj)
            assert(issorted(obj.vehicle_ids));

            % Initialize data readers/writers...
            % getenv('HOME'), 'dev/software/high_level_controller/examples/matlab' ...
            common_cpm_functions_path = fullfile( ...
                getenv('HOME'), 'dev/software/high_level_controller/examples/matlab' ...
            );
            assert(isfolder(common_cpm_functions_path), 'Missing folder "%s".', common_cpm_functions_path);
            addpath(common_cpm_functions_path);

            matlabDomainId = 1;
            [obj.matlabParticipant, obj.reader_vehicleStateList, obj.writer_vehicleCommandTrajectory, ~, obj.reader_systemTrigger, obj.writer_readyStatus, obj.trigger_stop, obj.writer_vehicleCommandDirect] = init_script(matlabDomainId); % #ok<ASGLU>

            % create Lab participant
            obj.matlabParticipantLab = DDS.DomainParticipant('MatlabLibrary::LocalCommunicationProfile', str2double(getenv('DDS_DOMAIN')));

            % create writer for lab visualization
            matlabVisualizationTopicName = 'visualization';
            obj.writer_visualization = DDS.DataWriter(DDS.Publisher(obj.matlabParticipantLab), 'Visualization', matlabVisualizationTopicName);

            % Set reader properties
            obj.reader_vehicleStateList.WaitSet = true;
            obj.reader_vehicleStateList.WaitSetTimeout = 5; % [s]

            % Middleware period for valid_after stamp
            obj.dt_period_nanos = uint64(obj.scenario.dt*1e9);

            % Sync start with infrastructure
            % Send ready signal for all assigned vehicle ids
            disp('Sending ready signal');
            for iVehicle = obj.vehicle_ids
                ready_msg = ReadyStatus;
                ready_msg.source_id = strcat('hlc_', num2str(iVehicle));
                ready_stamp = TimeStamp;
                ready_stamp.nanoseconds = uint64(0);
                ready_msg.next_start_stamp = ready_stamp;
                obj.writer_readyStatus.write(ready_msg);
            end

            % Wait for start or stop signal
            disp('Waiting for start or stop signal');
            
            got_start = false;
            got_stop = false;
            
            while (~got_stop && ~got_start)
                [got_start, got_stop] = read_system_trigger(obj.reader_systemTrigger, obj.trigger_stop);
            end

            % initialize ROS network to set rosrate
            if obj.scenario.options.firstManualVehicleMode == 2 || obj.scenario.options.secondManualVehicleMode == 2
                %rosinit;
            end
            
            % This command has first to be executed in terminal:
            %ros2 run joy joy_node _dev_name:="/dev/input/js0";
            if obj.scenario.manual_vehicle_id ~= 0
                if obj.scenario.options.firstManualVehicleMode == 1
                    obj.wheelNode = ros2node("/wheel");
                    obj.wheelSub = ros2subscriber(obj.wheelNode,"/j0");

                    % if function handle, then define ros types for pool
                    %obj.wheelNode = parallel.pool.Constant(ros2node("/wheel"));
                    %obj.wheelSub = parallel.pool.Constant(ros2subscriber(obj.wheelNode,"/j0"));
                    if obj.scenario.options.force_feedback_enabled
                        obj.g29_handler = G29ForceFeedback();
                        obj.g29_last_position = 0.0;
                    end
                elseif obj.scenario.options.firstManualVehicleMode == 2
                    %obj.wheelNode = ros2node("/wheel");
                    %obj.wheelSub = ros2subscriber(obj.wheelNode,"/j0","sensor_msgs/Joy",@obj.steeringWheelCallback);
                end
            end

            if obj.scenario.second_manual_vehicle_id ~= 0
                if obj.scenario.options.secondManualVehicleMode == 1
                    obj.gamepadNode = ros2node("/gamepad");
                    obj.gamepadSub = ros2subscriber(obj.gamepadNode,"/j1");
                elseif obj.scenario.options.secondManualVehicleMode == 2
                    obj.gamepadNode = ros2node("/gamepad");
                    obj.gamepadSub = ros2subscriber(obj.gamepadNode,"/j1", "sensor_msgs/Joy",@obj.gamepadCallback);
                end
            end
            
        end

        function wheelData = getWheelData(obj)
            wheelData = receive(obj.wheelSub, 1);
        end

        function steeringWheelCallback(obj, msg)
            
            wheel_message = obj.wheelSub.LatestMessage;
            wheelData = struct;
            wheelData.axes = wheel_message.axes;
            wheelData.buttons = wheel_message.buttons;
            wheelData.steering =  wheelData.axes(1);
            wheelData.throttle = wheelData.axes(3);
            wheelData.brake = wheelData.axes(4);
            wheelData.leftPaddle = wheelData.buttons(6);
            wheelData.rightPaddle = wheelData.buttons(5);
            
            %{
            if modeHandler.steering < -0.5
                modeHandler.steering = -0.5;
            elseif modeHandler.steering > 0.5
                modeHandler.steering = 0.5;
            end

            modeHandler.steering = 2.0 * modeHandler.steering;

            
            % make steering back to 0 easier
            if modeHandler.steering < 0 && obj.lastSteeringValue < modeHandler.steering
                modeHandler.steering = modeHandler.steering / 1.5;
            elseif modeHandler.steering > 0 && obj.lastSteeringValue > modeHandler.steering
                modeHandler.steering = modeHandler.steering / 1.5;
            end
            %}
        

            dt_max_comm_delay = uint64(100e6);
            if obj.dt_period_nanos >= dt_max_comm_delay
                dt_valid_after = obj.dt_period_nanos;
            else
                dt_valid_after = dt_max_comm_delay;
            end

            vehicle_command_direct = VehicleCommandDirect;
            vehicle_command_direct.vehicle_id = uint8(obj.scenario.manual_vehicle_id);

            throttle = 0;
            if wheelData.throttle >= 0
                throttle = wheelData.throttle;
            elseif wheelData.brake >= 0
                throttle = (-1) * wheelData.brake;
            end
            %{
            if wheelData.steering < -0.4
                wheelData.steering = -0.4;
            elseif wheelData.steering > 0.4
                wheelData.steering = 0.4;
            end
            %}
            wheelData.steering = 2.0 * wheelData.steering;

            if throttle > 0.4
                throttle = 0.4;
            elseif throttle < -0.4
                throttle = -0.4;
            end
            disp(sprintf("throttle: %f, steering: %f", throttle, wheelData.steering));
            vehicle_command_direct.motor_throttle = double(throttle);
            vehicle_command_direct.steering_servo = double(wheelData.steering);
            %vehicle_command_direct.header.create_stamp.nanoseconds = ...
                %uint64(timestamp);
            %vehicle_command_direct.header.valid_after_stamp.nanoseconds = ...
                %uint64(timestamp) + uint64(dt_valid_after);
            vehicle_command_direct.header.create_stamp.nanoseconds = ...
                uint64(obj.sample.t_now);
            vehicle_command_direct.header.valid_after_stamp.nanoseconds = ...
                uint64(obj.sample.t_now+dt_valid_after);
            %obj.writer_vehicleCommandDirect.write(vehicle_command_direct);
        end

        function WriteAsyncFcn(obj, msg)
            global stored_wheel_messages_global
            writeasync(obj.stored_wheel_msgs, stored_wheel_messages_global);
        end

        function wheelData = get_stored_wheel_msgs(obj)
            global stored_wheel_messages_global
            obj.stored_wheel_msgs = stored_wheel_messages_global;
            wheelData = obj.stored_wheel_msgs;
        end

        function gamepadData = getGamepadData(obj)
            gamepadData = receive(obj.gamepadSub, 1);
        end

        function gamepadCallback(obj, msg)    
            global stored_gamepad_messages_global

            stored_gamepad_messages_global = msg;
        end

        function gamepadData = get_stored_gamepad_msgs(obj)
            global stored_gamepad_messages_global
            obj.stored_gamepad_msgs = stored_gamepad_messages_global;
            gamepadData = obj.stored_gamepad_msgs;
        end
        
        function [ x0, trim_indices ] = measure(obj, controller_init)
            [obj.sample, ~, sample_count, ~] = obj.reader_vehicleStateList.take();
            if (sample_count > 1)
                warning('Received %d samples, expected 1. Correct middleware period? Missed deadline?', sample_count);
            end
            
            % for first iteration use real poses
            if controller_init == false
                x0 = zeros(obj.scenario.nVeh,4);
                pose = [obj.sample(end).state_list.pose];
                x0(:,1) = [pose.x];
                x0(:,2) = [pose.y];
                x0(:,3) = [pose.yaw];
                x0(:,4) = [obj.sample(end).state_list.speed];
                [ ~, trim_indices ] = obj.measure_node();
            else
                [ x0, trim_indices ] = obj.measure_node();

                % find out index of vehicle in Expert-Mode
                indexVehicleExpertMode = 0;
                for j = 1:obj.scenario.nVeh
                    if ((obj.scenario.vehicle_ids(j) == obj.scenario.manual_vehicle_id && obj.scenario.options.firstManualVehicleMode == 2) ...
                        || (obj.scenario.vehicle_ids(j) == obj.scenario.second_manual_vehicle_id && obj.scenario.options.secondManualVehicleMode == 2))
                        indexVehicleExpertMode = j;
                    end
                end

                % use real poses for vehicle in Expert Mode
                if indexVehicleExpertMode ~= 0
                    pose = [obj.sample(end).state_list.pose];
                    x0(indexVehicleExpertMode,1) = [pose(1,indexVehicleExpertMode).x];
                    x0(indexVehicleExpertMode,2) = [pose(1,indexVehicleExpertMode).y];
                    x0(indexVehicleExpertMode,3) = [pose(1,indexVehicleExpertMode).yaw];
                    x0(indexVehicleExpertMode,4) = [obj.sample(end).state_list(1,indexVehicleExpertMode).speed];
                end
            end
        end
        
        function apply(obj, info, ~, k, scenario)
            y_pred = info.y_predicted;
            % simulate change of state
            obj.cur_node = info.next_node;
            obj.k = k;
            % calculate vehicle control messages
            obj.out_of_map_limits = false(obj.scenario.nVeh,1);
            for iVeh = 1:obj.scenario.nVeh
                n_traj_pts = obj.scenario.Hp;
                n_predicted_points = size(y_pred{iVeh},1);
                idx_predicted_points = 1:n_predicted_points/n_traj_pts:n_predicted_points;
                trajectory_points(1:n_traj_pts) = TrajectoryPoint;
                for i_traj_pt = 1:n_traj_pts
                    i_predicted_points = idx_predicted_points(i_traj_pt);
                    trajectory_points(i_traj_pt).t.nanoseconds = ...
                        uint64(obj.sample(end).t_now + i_traj_pt*obj.dt_period_nanos);
                    trajectory_points(i_traj_pt).px = y_pred{iVeh}(i_predicted_points,1);
                    trajectory_points(i_traj_pt).py = y_pred{iVeh}(i_predicted_points,2);
                
                    yaw = y_pred{iVeh}(i_predicted_points,3);

                    if ((scenario.vehicle_ids(iVeh) == scenario.manual_vehicle_id) && scenario.manual_mpa_initialized) ...
                        || ((scenario.vehicle_ids(iVeh) == scenario.second_manual_vehicle_id) && scenario.second_manual_mpa_initialized)
                        speed = scenario.vehicles(iVeh).vehicle_mpa.trims(y_pred{iVeh}(i_predicted_points,4)).speed;
                    else
                        speed = obj.scenario.mpa.trims(y_pred{iVeh}(i_predicted_points,4)).speed;
                    end
                    
                    trajectory_points(i_traj_pt).vx = cos(yaw)*speed;
                    trajectory_points(i_traj_pt).vy = sin(yaw)*speed;
                end

                is_manual_vehicle = (scenario.vehicle_ids(iVeh) == scenario.manual_vehicle_id && scenario.options.firstManualVehicleMode == 2) ...
                    || ((scenario.vehicle_ids(iVeh) == scenario.second_manual_vehicle_id) && scenario.options.secondManualVehicleMode == 2);
                    
                if (is_manual_vehicle)
                    obj.out_of_map_limits(iVeh) = false;
                else
                    obj.out_of_map_limits(iVeh) = obj.is_veh_at_map_border(trajectory_points);
                end
                
                if (is_manual_vehicle)
                    vehicle_command_trajectory = VehicleCommandTrajectory;
                    vehicle_command_trajectory.vehicle_id = uint8(obj.vehicle_ids(iVeh));
                    vehicle_command_trajectory.trajectory_points = [];
                    vehicle_command_trajectory.header.create_stamp.nanoseconds = ...
                        uint64(obj.sample(end).t_now);
                    vehicle_command_trajectory.header.valid_after_stamp.nanoseconds = ...
                        uint64(obj.sample(end).t_now + obj.dt_period_nanos);
                else
                    vehicle_command_trajectory = VehicleCommandTrajectory;
                    vehicle_command_trajectory.vehicle_id = uint8(obj.vehicle_ids(iVeh));
                    vehicle_command_trajectory.trajectory_points = trajectory_points;
                    vehicle_command_trajectory.header.create_stamp.nanoseconds = ...
                        uint64(obj.sample(end).t_now);
                    vehicle_command_trajectory.header.valid_after_stamp.nanoseconds = ...
                        uint64(obj.sample(end).t_now + obj.dt_period_nanos);

                    if (scenario.vehicle_ids(iVeh) == scenario.manual_vehicle_id)
                        if scenario.updated_manual_vehicle_path || obj.visualize_manual_lane_change_counter > 0
    
                            [visualization_command_line] = lab_visualize_lane_change(scenario, trajectory_points, iVeh, true);
                            obj.writer_visualization.write(visualization_command_line);

                            if obj.visualize_manual_lane_change_counter == 0
                                obj.visualize_manual_lane_change_counter = 4;
                            else
                                obj.visualize_manual_lane_change_counter = obj.visualize_manual_lane_change_counter - 1;
                            end
                        end
                    elseif (scenario.vehicle_ids(iVeh) == scenario.second_manual_vehicle_id)
                        if scenario.updated_second_manual_vehicle_path || obj.visualize_second_manual_lane_change_counter > 0

                            [visualization_command_line] = lab_visualize_lane_change(scenario, trajectory_points, iVeh, true);
                            obj.writer_visualization.write(visualization_command_line);

                            if obj.visualize_second_manual_lane_change_counter == 0
                                obj.visualize_second_manual_lane_change_counter = 4;
                            else
                                obj.visualize_second_manual_lane_change_counter = obj.visualize_second_manual_lane_change_counter - 1;
                            end
                        end
                    end
                end

                obj.writer_vehicleCommandTrajectory.write(vehicle_command_trajectory);

                if obj.scenario.options.force_feedback_enabled && scenario.vehicle_ids(iVeh) == scenario.manual_vehicle_id
                    obj.g29_last_position = obj.g29_handler.g29_send_message(obj.sample(end).state_list(iVeh).steering_servo, 0.3, obj.g29_last_position);                 
                end
            end
        end

        function visualize(obj, visualization_command)
            obj.writer_visualization.write(visualization_command);
        end

        function updateManualControl(obj, modeHandler, scenario, vehicle_id, steeringWheel, timestamp)
            
            dt_max_comm_delay = uint64(100e6);
            if obj.dt_period_nanos >= dt_max_comm_delay
                dt_valid_after = obj.dt_period_nanos;
            else
                dt_valid_after = dt_max_comm_delay;
            end

            vehicle_command_direct = VehicleCommandDirect;
            vehicle_command_direct.vehicle_id = uint8(scenario.manual_vehicle_id);

            throttle = 0;
            if modeHandler.throttle >= 0
                throttle = modeHandler.throttle;
            elseif modeHandler.brake >= 0
                throttle = (-1) * modeHandler.brake;
            end
            disp(sprintf("throttle: %f, steering: %f", throttle, modeHandler.steering));
            vehicle_command_direct.motor_throttle = double(throttle);
            vehicle_command_direct.steering_servo = double(modeHandler.steering);
            %vehicle_command_direct.header.create_stamp.nanoseconds = ...
                %uint64(timestamp);
            %vehicle_command_direct.header.valid_after_stamp.nanoseconds = ...
                %uint64(timestamp) + uint64(dt_valid_after);
            vehicle_command_direct.header.create_stamp.nanoseconds = ...
                uint64(obj.sample.t_now);
            vehicle_command_direct.header.valid_after_stamp.nanoseconds = ...
                uint64(obj.sample.t_now+dt_valid_after);
            %vehicle_command_direct.header.valid_after_stamp.nanoseconds = ...
                %uint64(obj.sample.t_now+dt);
            obj.writer_vehicleCommandDirect.write(vehicle_command_direct);
        end
        
        function got_stop = is_stop(obj)
            [~, got_stop] = read_system_trigger(obj.reader_systemTrigger, obj.trigger_stop);
            if any(obj.out_of_map_limits)
                got_stop = true;
            end
        end
        
        function end_run(obj)
            disp('End')   
        end
        
    end
    
    methods (Access=private)    
        % helper function
        function stop_experiment = is_veh_at_map_border(obj, trajectory_points)
            % Vehicle command timeout is 1000 ms after the last valid_after_stamp,
            % so vehicle initiates stop between third and fourth trajectory point
            vhlength = 0.25;
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
    end
end

