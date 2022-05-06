classdef CPMLab < InterfaceExperiment
% CPMLAB    Instance of experiment interface for usage in the cpm lab.
    
    properties (Access=private)
        vehicle_ids
        matlabParticipant
        reader_vehicleStateList
        writer_vehicleCommandTrajectory
        reader_systemTrigger
        writer_readyStatus
        trigger_stop
        controller_init
        dt_period_nanos
        sample
        out_of_map_limits
        wheelNode
        wheelSub
    end
    
    methods
        function obj = CPMLab(scenario, vehicle_ids)
            obj.vehicle_ids = vehicle_ids;
            obj.scenario = scenario;
            obj.controller_init = false;
            obj.cur_node = node(0, [obj.scenario.vehicles(:).trim_config], [obj.scenario.vehicles(:).x_start]', [obj.scenario.vehicles(:).y_start]', [obj.scenario.vehicles(:).yaw_start]', zeros(obj.scenario.nVeh,1), zeros(obj.scenario.nVeh,1));
        end

        function steeringWheelCallback(src, message)    
            global wheelAxes
            global wheelButtons
            
            % Extract Axes and Buttons from the ROS message and assign the
            % data to the global variables.
            wheelAxes = message.axes;
            wheelButtons = message.buttons;
        end
        
        
        function setup(obj)
            assert(issorted(obj.vehicle_ids));

            %obj.joy = vrjoystick(0);
            %test1 = ros2node("/test1");
            %ros2 node list;
            %exampleHelperROS2CreateSampleNetwork

            % This command has first to be executed in terminal:
            %ros2 run joy joy_node _dev_name:="/dev/input/js0";
            obj.wheelNode = ros2node("/wheel");
            obj.wheelSub = ros2subscriber(obj.wheelNode,"/joy",@obj.steeringWheelCallback);

            % Initialize data readers/writers...
            % getenv('HOME'), 'dev/software/high_level_controller/examples/matlab' ...
            common_cpm_functions_path = fullfile( ...
                getenv('HOME'), 'dev/software/high_level_controller/examples/matlab' ...
            );
            assert(isfolder(common_cpm_functions_path), 'Missing folder "%s".', common_cpm_functions_path);
            addpath(common_cpm_functions_path);

            matlabDomainId = 1;
            [obj.matlabParticipant, obj.reader_vehicleStateList, obj.writer_vehicleCommandTrajectory, ~, obj.reader_systemTrigger, obj.writer_readyStatus, obj.trigger_stop] = init_script(matlabDomainId); % #ok<ASGLU>

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
        end

        function wheelData = getWheelData(obj)
            wheelData = receive(obj.wheelSub, 1);
        end

        function update(obj)
            obj.cur_node = node(0, [obj.scenario.vehicles(:).trim_config], [obj.scenario.vehicles(:).x_start]', [obj.scenario.vehicles(:).y_start]', [obj.scenario.vehicles(:).yaw_start]', zeros(obj.scenario.nVeh,1), zeros(obj.scenario.nVeh,1));
        end
        
        function [ x0, trim_indices ] = measure(obj)
            [obj.sample, ~, sample_count, ~] = obj.reader_vehicleStateList.take();
            if (sample_count > 1)
                warning('Received %d samples, expected 1. Correct middleware period? Missed deadline?', sample_count);
            end
            
            % for first iteration use real poses
            if obj.controller_init == false
                x0 = zeros(obj.scenario.nVeh,4);
                pose = [obj.sample(end).state_list.pose];
                x0(:,1) = [pose.x];
                x0(:,2) = [pose.y];
                %{
                x0(:,1) = [pose.y];
                % WARNING: wrong x and y coordinates are inverted, and new x (former y) coordinate has to be shifted
                x0(:,2) = [pose.x];

                sub = zeros(obj.scenario.nVeh,4);
                for i = 1:obj.scenario.nVeh
                    sub(i,1) = 4;
                end

                for i = 1:obj.scenario.nVeh
                    x0(i,1) = sub(i,1) - x0(i,1);
                end
                %}

                for i = 1:obj.scenario.nVeh
                    %disp(obj.scenario.nVeh);
                    disp("x: ");
                    disp(x0(i));
                    disp("y: ");
                    disp(x0(i,2));
                end
                
                x0(:,3) = [pose.yaw];
                x0(:,4) = [obj.sample(end).state_list.speed];
                obj.controller_init = true;
                [ ~, trim_indices ] = obj.measure_node();
            else
                [ x0, trim_indices ] = obj.measure_node();
            end
        end
        
        function apply(obj, ~, y_pred, info, ~, k)
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
                    %{
                    trajectory_points(i_traj_pt).px = y_pred{iVeh}(i_predicted_points,2);
                    disp("x: ");
                    disp(trajectory_points(i_traj_pt).px);
                    trajectory_points(i_traj_pt).py = y_pred{iVeh}(i_predicted_points,1);
                    trajectory_points(i_traj_pt).py = 4 - trajectory_points(i_traj_pt).py;
                    disp("y: ");
                    disp(trajectory_points(i_traj_pt).py);
                    %}
                    yaw = y_pred{iVeh}(i_predicted_points,3);
                    speed = obj.scenario.mpa.trims(y_pred{iVeh}(i_predicted_points,4)).speed;
                    trajectory_points(i_traj_pt).vx = cos(yaw)*speed;
                    trajectory_points(i_traj_pt).vy = sin(yaw)*speed;
                end
                obj.out_of_map_limits(iVeh) = obj.is_veh_at_map_border(trajectory_points);
                vehicle_command_trajectory = VehicleCommandTrajectory;
                vehicle_command_trajectory.vehicle_id = uint8(obj.vehicle_ids(iVeh));
                vehicle_command_trajectory.trajectory_points = trajectory_points;
                vehicle_command_trajectory.header.create_stamp.nanoseconds = ...
                    uint64(obj.sample(end).t_now);
                vehicle_command_trajectory.header.valid_after_stamp.nanoseconds = ...
                    uint64(obj.sample(end).t_now + obj.dt_period_nanos);
                obj.writer_vehicleCommandTrajectory.write(vehicle_command_trajectory);
            end
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

