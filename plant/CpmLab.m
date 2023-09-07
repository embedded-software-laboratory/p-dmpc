classdef CpmLab < Plant
    % CPMLAB    Instance of experiment interface for usage in the cpm lab.

    properties (Access = private)
        dds_participant
        dds_participant_lab
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
        pos_init
    end

    methods

        function obj = CpmLab()
            obj = obj@Plant();
            obj.pos_init = false;

            if ispc
                error('You are using a Windows machine, please do not select lab mode!')
            end

        end

        function setup(obj, scenario, ~)

            arguments
                obj (1, 1) CpmLab
                scenario (1, 1) Scenario
                ~
            end

            % Initialize data readers/writers...
            % getenv('HOME'), 'dev/software/high_level_controller/examples/matlab' ...
            common_cpm_functions_path = fullfile( ...
                getenv('HOME'), 'dev/software/high_level_controller/examples/matlab' ...
            );
            assert(isfolder(common_cpm_functions_path), 'Missing folder "%s".', common_cpm_functions_path);
            addpath(common_cpm_functions_path);

            matlabDomainId = 1;
            [obj.dds_participant, obj.reader_vehicleStateList, obj.writer_vehicleCommandTrajectory, ~, obj.reader_systemTrigger, obj.writer_readyStatus, obj.trigger_stop, obj.writer_vehicleCommandDirect] = init_script(matlabDomainId); % #ok<ASGLU>

            % create Lab participant
            obj.dds_participant_lab = DDS.DomainParticipant('MatlabLibrary::LocalCommunicationProfile', str2double(getenv('DDS_DOMAIN')));

            % create writer for lab visualization
            matlabVisualizationTopicName = 'visualization';
            obj.writer_visualization = DDS.DataWriter(DDS.Publisher(obj.dds_participant_lab), 'Visualization', matlabVisualizationTopicName);

            % Set reader properties
            obj.reader_vehicleStateList.WaitSet = true;
            obj.reader_vehicleStateList.WaitSetTimeout = 5; % [s]

            % get middleware period and vehicle ids from vehicle state list message
            [sample, ~, sample_count, ~] = obj.reader_vehicleStateList.take();

            if (sample_count == 0)
                error('No vehicle state list received during CpmLab.setup!');
            end

            state_list = sample(end);

            % Middleware period for valid_after stamp
            obj.dt_period_nanos = uint64(scenario.options.dt * 1e9);

            setup@Plant(obj, scenario, state_list.active_vehicle_ids);
        end

        function [x0, trim_indices] = measure(obj)
            [obj.sample, ~, sample_count, ~] = obj.reader_vehicleStateList.take();

            if (sample_count > 1)
                warning('Received %d samples, expected 1. Correct middleware period? Missed deadline?', sample_count);
            end

            state_list = obj.sample(end).state_list;

            x0 = zeros(obj.scenario.options.amount + obj.scenario.options.manual_control_config.amount, 4);

            % for first iteration use real poses
            if (obj.pos_init == false)
                cav_index = 1;

                for index = 1:length(state_list)

                    if ismember(state_list(index).vehicle_id, obj.controlled_vehicle_ids) % measure cav states
                        list_index = obj.indices_in_vehicle_list(cav_index); % use list to prevent breaking distributed control
                        cav_index = cav_index + 1;
                        x0(list_index, 1) = state_list(index).pose.x;
                        x0(list_index, 2) = state_list(index).pose.y;
                        x0(list_index, 3) = state_list(index).pose.yaw;
                        x0(list_index, 4) = [state_list(index).speed];
                    end

                end

                [~, trim_indices] = obj.measure_node();
                obj.pos_init = true;
            else
                [x0(1:obj.scenario.options.amount, :), trim_indices] = obj.measure_node(); % get cav states from current node
            end

            % Always measure HDV
            hdv_index = 1;

            for index = 1:length(state_list)

                if ismember(state_list(index).vehicle_id, obj.scenario.options.manual_control_config.hdv_ids)
                    list_index = obj.scenario.options.amount + hdv_index;
                    hdv_index = hdv_index + 1;
                    x0(list_index, 1) = state_list(index).pose.x;
                    x0(list_index, 2) = state_list(index).pose.y;
                    x0(list_index, 3) = state_list(index).pose.yaw;
                    x0(list_index, 4) = [state_list(index).speed];
                end

            end

        end

        function apply(obj, info, ~, k, scenario)
            y_pred = info.y_predicted;
            % simulate change of state
            for iVeh = obj.indices_in_vehicle_list
                obj.cur_node(iVeh, :) = info.next_node(iVeh, :);
            end

            obj.k = k;
            % calculate vehicle control messages
            obj.out_of_map_limits = false(obj.scenario.options.amount, 1);

            for iVeh = obj.indices_in_vehicle_list
                n_traj_pts = obj.scenario.options.Hp;
                n_predicted_points = size(y_pred{iVeh}, 1);
                idx_predicted_points = 1:n_predicted_points / n_traj_pts:n_predicted_points;
                trajectory_points(1:n_traj_pts) = TrajectoryPoint;

                for i_traj_pt = 1:n_traj_pts
                    i_predicted_points = idx_predicted_points(i_traj_pt);
                    trajectory_points(i_traj_pt).t.nanoseconds = ...
                        uint64(obj.sample(end).t_now + i_traj_pt * obj.dt_period_nanos);
                    trajectory_points(i_traj_pt).px = y_pred{iVeh}(i_predicted_points, 1);
                    trajectory_points(i_traj_pt).py = y_pred{iVeh}(i_predicted_points, 2);

                    yaw = y_pred{iVeh}(i_predicted_points, 3);

                    speed = scenario.mpa.trims(y_pred{iVeh}(i_predicted_points, 4)).speed;

                    trajectory_points(i_traj_pt).vx = cos(yaw) * speed;
                    trajectory_points(i_traj_pt).vy = sin(yaw) * speed;
                end

                obj.out_of_map_limits(iVeh) = obj.is_veh_at_map_border(trajectory_points);

                vehicle_command_trajectory = VehicleCommandTrajectory;
                vehicle_command_trajectory.vehicle_id = uint8(obj.all_vehicle_ids(iVeh));
                vehicle_command_trajectory.trajectory_points = trajectory_points;
                vehicle_command_trajectory.header.create_stamp.nanoseconds = ...
                    uint64(obj.sample(end).t_now);
                vehicle_command_trajectory.header.valid_after_stamp.nanoseconds = ...
                    uint64(obj.sample(end).t_now + obj.dt_period_nanos + 1);

                obj.writer_vehicleCommandTrajectory.write(vehicle_command_trajectory);
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

        function sychronize_start_with_plant(obj)
            % Sync start with infrastructure
            % Send ready signal for all assigned vehicle ids
            disp('Sending ready signal');

            for iVehicle = sort([obj.controlled_vehicle_ids, obj.scenario.options.manual_control_config.hdv_ids])
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

    end

    methods (Access = private)
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

    end

end
