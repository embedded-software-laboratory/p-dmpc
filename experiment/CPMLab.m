classdef CPMLab < InterfaceExperiment
% CPMLAB    Instance of experiment interface for usage in the cpm lab.
    
    properties (Access=private)
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
        g29_handler
        g29_last_position
        pos_init
        indices_in_vehicle_list
    end
    
    methods
        function obj = CPMLab(scenario, veh_ids)
            obj = obj@InterfaceExperiment(veh_ids);
            obj.scenario = scenario;
            obj.amount = size(obj.veh_ids);
            obj.pos_init = false;
            obj.visualize_manual_lane_change_counter = 0;
            obj.visualize_second_manual_lane_change_counter = 0;
            obj.cur_node = node(0, [obj.scenario.vehicles(:).trim_config], [obj.scenario.vehicles(:).x_start]', [obj.scenario.vehicles(:).y_start]', [obj.scenario.vehicles(:).yaw_start]', zeros(amount,1), zeros(amount,1));
            if obj.amount == 1
                obj.indices_in_vehicle_list = [find(scenario.options.veh_ids == obj.vehicle_ids(1),1)];
            else
                obj.indices_in_vehicle_list = 1:obj.amount;
            end
        end
        
        function setup(obj)
            assert(issorted(obj.veh_ids));

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
            obj.dt_period_nanos = uint64(obj.scenario.options.dt*1e9);

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
            
            if obj.scenario.manual_vehicle_id ~= 0
                if obj.scenario.options.firstManualVehicleMode == 1
                    % setup subscriber for joy package
                    obj.wheelNode = ros2node("/wheel");
                    obj.wheelSub = ros2subscriber(obj.wheelNode,"/j0");

                    % setup handler to rotate wheel to specified position
                    obj.g29_handler = G29ForceFeedback();
                    obj.g29_last_position = 0.01;
                else
                    pathToScript = fullfile(pwd,'/experiment','expert_mode_script.sh');
                    if obj.scenario.options.force_feedback_enabled
                        feedback = 1;
                    else
                        feedback = 0;
                    end

                    % ------------------------------------------
                    % comment out both following lines to disable automatic execution of Expert Mode
                    cmdStr = ['gnome-terminal --' ' ' pathToScript ' ' num2str(obj.scenario.manual_vehicle_id) ' ' num2str(feedback)];
                    system(cmdStr);
                    % ------------------------------------------
                end
            end

            if obj.scenario.second_manual_vehicle_id ~= 0
                obj.gamepadNode = ros2node("/gamepad");
                obj.gamepadSub = ros2subscriber(obj.gamepadNode,"/j1");
            end    
        end

        function wheelData = getWheelData(obj)
            wheelData = receive(obj.wheelSub, 1);
        end

        function gamepadData = getGamepadData(obj)
            gamepadData = receive(obj.gamepadSub, 1);
        end

        function [ x0, trim_indices ] = measure(obj)
            [obj.sample, ~, sample_count, ~] = obj.reader_vehicleStateList.take();
            if (sample_count > 1)
                warning('Received %d samples, expected 1. Correct middleware period? Missed deadline?', sample_count);
            end
            
            % for first iteration use real poses
            if obj.pos_init == false
                x0 = zeros(obj.scenario.options.amount,4);
                pose = [obj.sample(end).state_list.pose];
                for index = obj.indices_in_vehicle_list
                    x0(index,1) = pose.x(index);
                    x0(index,2) = pose.y(index);
                    x0(index,3) = pose.yaw(index);
                    x0(index,4) = [obj.sample(end).state_list.speed(index)];
                end
                [ ~, trim_indices ] = obj.measure_node();
                obj.pos_init = true;
            else
                [ x0, trim_indices ] = obj.measure_node();

                % find out index of vehicle in Expert-Mode
                indexVehicleExpertMode = 0;
                for j = 1:obj.scenario.options.amount
                    if ((obj.scenario.options.veh_ids(j) == obj.scenario.manual_vehicle_id && obj.scenario.options.firstManualVehicleMode == 2) ...
                        || (obj.scenario.options.veh_ids(j) == obj.scenario.second_manual_vehicle_id && obj.scenario.options.secondManualVehicleMode == 2))
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
            obj.out_of_map_limits = false(obj.scenario.options.amount,1);
            for iVeh = 1:obj.amount
                n_traj_pts = obj.scenario.options.Hp;
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

                    if ((scenario.options.veh_ids(iVeh) == scenario.manual_vehicle_id) && scenario.manual_mpa_initialized) ...
                        || ((scenario.options.veh_ids(iVeh) == scenario.second_manual_vehicle_id) && scenario.second_manual_mpa_initialized)
                        speed = scenario.vehicles(iVeh).vehicle_mpa.trims(y_pred{iVeh}(i_predicted_points,4)).speed;
                    else
                        speed = obj.scenario.mpa.trims(y_pred{iVeh}(i_predicted_points,4)).speed;
                    end
                    
                    trajectory_points(i_traj_pt).vx = cos(yaw)*speed;
                    trajectory_points(i_traj_pt).vy = sin(yaw)*speed;
                end

                is_manual_vehicle_expert_mode = (scenario.options.veh_ids(iVeh) == scenario.manual_vehicle_id && scenario.options.firstManualVehicleMode == 2) ...
                    || ((scenario.options.veh_ids(iVeh) == scenario.second_manual_vehicle_id) && scenario.options.secondManualVehicleMode == 2);
                    
                if (is_manual_vehicle_expert_mode)
                    obj.out_of_map_limits(iVeh) = false;
                else
                    obj.out_of_map_limits(iVeh) = obj.is_veh_at_map_border(trajectory_points);
                end
                
                if ~is_manual_vehicle_expert_mode
                    vehicle_command_trajectory = VehicleCommandTrajectory;
                    vehicle_command_trajectory.vehicle_id = uint8(obj.veh_ids(iVeh));
                    vehicle_command_trajectory.trajectory_points = trajectory_points;
                    vehicle_command_trajectory.header.create_stamp.nanoseconds = ...
                        uint64(obj.sample(end).t_now);
                    vehicle_command_trajectory.header.valid_after_stamp.nanoseconds = ...
                        uint64(obj.sample(end).t_now + obj.dt_period_nanos + 1);

                    if (scenario.options.veh_ids(iVeh) == scenario.manual_vehicle_id)
                        if scenario.updated_manual_vehicle_path || obj.visualize_manual_lane_change_counter > 0
    
                            [visualization_command_line] = lab_visualizer(trajectory_points, 'laneChange');
                            obj.writer_visualization.write(visualization_command_line);

                            if obj.visualize_manual_lane_change_counter == 0
                                obj.visualize_manual_lane_change_counter = 4;
                            else
                                obj.visualize_manual_lane_change_counter = obj.visualize_manual_lane_change_counter - 1;
                            end
                        end

                        if scenario.options.visualizeReferenceTrajectory
                            [visualization_command_line] = lab_visualizer(scenario.vehicles(iVeh).referenceTrajectory, 'referenceTrajectory');
                            obj.writer_visualization.write(visualization_command_line);
                        end

                    elseif (scenario.options.veh_ids(iVeh) == scenario.second_manual_vehicle_id)
                        if scenario.updated_second_manual_vehicle_path || obj.visualize_second_manual_lane_change_counter > 0

                            [visualization_command_line] = lab_visualizer(trajectory_points, 'laneChange');
                            obj.writer_visualization.write(visualization_command_line);

                            if obj.visualize_second_manual_lane_change_counter == 0
                                obj.visualize_second_manual_lane_change_counter = 4;
                            else
                                obj.visualize_second_manual_lane_change_counter = obj.visualize_second_manual_lane_change_counter - 1;
                            end
                        end

                        if scenario.options.visualizeReferenceTrajectory
                            [visualization_command_line] = lab_visualizer(scenario.vehicles(iVeh).referenceTrajectory, 'referenceTrajectory');
                            obj.writer_visualization.write(visualization_command_line);
                        end
                    end

                    obj.writer_vehicleCommandTrajectory.write(vehicle_command_trajectory);
                end

                if  scenario.options.veh_ids(iVeh) == scenario.manual_vehicle_id && obj.scenario.options.firstManualVehicleMode == 1
                    if obj.scenario.options.force_feedback_enabled
                        obj.g29_last_position = obj.g29_handler.g29_send_message(obj.sample(end).state_list(iVeh).steering_servo, 0.3, obj.g29_last_position); 
                    else
                        obj.g29_last_position = obj.g29_handler.g29_send_message(0.01, 0.3, obj.g29_last_position);
                    end
                end
            end
        end

        function visualize(obj, visualization_command)
            obj.writer_visualization.write(visualization_command);
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

