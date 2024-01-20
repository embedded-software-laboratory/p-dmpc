classdef CpmLab < Plant
    % CPMLAB    Instance of experiment interface for usage in the cpm lab.

    properties (Access = private)
        dds_participant
        reader_vehicleStateList
        writer_vehicleCommandTrajectory
        writer_vehicleCommandDirect
        reader_systemTrigger
        writer_readyStatus
        trigger_stop

        dt_period_nanos
        sample
        out_of_map_limits

        time_now (1, 1) uint64 = 0;
        init_completed = false
    end

    properties (GetAccess = public, SetAccess = private)
        % all active vehicle ids. when running distributed, these can differ from controlled ids
        all_vehicle_ids (1, :) uint8 % uint8 to conform to ids sent by the middleware
    end

    properties (Dependent, GetAccess = public)
        % which vehicles will controlled by this experiment instance
        vehicle_ids_controlled (1, :) uint8
    end

    methods

        % get methods for dependent properties
        function ids = get.vehicle_ids_controlled(obj)
            ids = obj.all_vehicle_ids(obj.vehicle_indices_controlled);
        end

    end

    methods

        function obj = CpmLab()
            obj = obj@Plant();
        end

        function setup(obj, options, vehicle_indices_controlled)

            arguments
                obj (1, 1) CpmLab
                options (1, 1) Config
                vehicle_indices_controlled (1, :) uint8
            end

            % create data readers/writers...
            obj.prepare_dds();

            % get middleware period and vehicle ids from vehicle state list message
            [initial_samples, ~, sample_count, ~] = obj.reader_vehicleStateList.take();

            if (sample_count == 0)
                error('No vehicle state list received during CpmLab.setup!');
            end

            sample_for_setup = initial_samples(end);

            % sample_for_setup.period_ms
            % data type unsigned long long (IDL)
            % aka uint64_t (C++) aka uint64 (Matlab)

            % take step time from sample_for_setup
            options.dt_seconds = cast(sample_for_setup.period_ms, "double") / 1e3;

            % middleware period for valid_after stamp
            obj.dt_period_nanos = uint64(options.dt_seconds * 1e9);

            % validate the amount of active_vehicle_ids
            assert( ...
                length(sample_for_setup.active_vehicle_ids) == ...
                options.amount + options.manual_control_config.amount, ...
                'Amount of active_vehicle_ids (%d) does not match expected amount (%d)!', ...
                length(sample_for_setup.active_vehicle_ids), ...
                options.amount + options.manual_control_config.amount ...
            );

            % subtract the hdv_ids from active_vehicle_ids
            obj.all_vehicle_ids = setdiff( ...
                cast(sample_for_setup.active_vehicle_ids, "uint8"), ...
                options.manual_control_config.hdv_ids ...
            );

            obj.vehicle_indices_controlled = vehicle_indices_controlled;

            setup@Plant(obj, options);

            % take list of VehicleStates from sample
            state_list = sample_for_setup.state_list;

            % since there is no steering info in [rad],
            % the initial_steering is assumed to 0
            initial_steering = 0;

            for index = 1:length(state_list)

                % data type of state_list vehicle_id is uint8 and
                % matches the data type of the superclass variable
                % casting of the data type is not necessary
                if ~any(state_list(index).vehicle_id == obj.vehicle_ids_controlled)
                    % if vehicle is not controlled, do not use received information
                    continue
                end

                % boolean with exactly one true entry for the position in all_vehicle_ids
                is_in_vehicle_list = obj.all_vehicle_ids == state_list(index).vehicle_id;

                obj.measurements(is_in_vehicle_list) = PlantMeasurement( ...
                    state_list(index).pose.x, ...
                    state_list(index).pose.y, ...
                    state_list(index).pose.yaw, ...
                    state_list(index).speed, ...
                    initial_steering ...
                );

            end

        end

        function synchronize_start_with_plant(obj)
            % Sync start with infrastructure
            disp('Sending ready signal, waiting for start or stop signal');

            got_start = false;
            got_stop = false;

            while (~got_stop && ~got_start)

                for iVehicle = obj.vehicle_ids_controlled
                    ready_msg = ReadyStatus;
                    ready_msg.source_id = strcat('hlc_', num2str(iVehicle));
                    ready_stamp = TimeStamp;
                    ready_stamp.nanoseconds = uint64(0);
                    ready_msg.next_start_stamp = ready_stamp;
                    obj.writer_readyStatus.write(ready_msg);
                end

                [got_start, got_stop] = read_system_trigger(obj.reader_systemTrigger, obj.trigger_stop);

            end

            obj.time_now = 0;
            obj.init_completed = true;

        end

        function [cav_measurements, hdv_measurements] = measure(obj)
            % take cav_measurements that were applied in the previous step
            % for the first step they hold the initialized values
            cav_measurements = obj.measurements;

            if ~obj.time_now || ~obj.init_completed
                [obj.sample, ~, sample_count, ~] = obj.reader_vehicleStateList.take();
                obj.time_now = uint64(obj.sample(end).t_now);

                if (sample_count > 1)
                    warning('Received %d samples, expected 1. Correct middleware period? Missed deadline?', sample_count);
                end

            else
                obj.time_now = obj.time_now + uint64(obj.dt_period_nanos);
            end

            % if there are no manual vehicles return directly
            if ~obj.manual_control_config.is_active
                hdv_measurements = [];
                return
            end

            state_list = obj.sample(end).state_list;

            % initialize return variable
            hdv_measurements(obj.manual_control_config.amount, 1) = PlantMeasurement();

            % since there is no steering info in [rad],
            % the hdv_steering is assumed to 0
            hdv_steering = 0;

            for index = 1:length(state_list)

                % data type of state_list vehicle_id is uint8 and
                % matches the data type of the manual_control_config member
                % casting of the data type is not necessary
                if ~any(state_list(index).vehicle_id == obj.manual_control_config.hdv_ids)
                    % if vehicle is no hdv, do not use received information
                    continue
                end

                % boolean with exactly one true entry for the position in hdv_ids
                is_in_hdv_list = state_list(index).vehicle_id == obj.manual_control_config.hdv_ids;

                hdv_measurements(is_in_hdv_list) = PlantMeasurement( ...
                    state_list(index).pose.x, ...
                    state_list(index).pose.y, ...
                    state_list(index).pose.yaw, ...
                    state_list(index).speed, ...
                    hdv_steering ...
                );

            end

        end

        function apply(obj, info, ~, ~, mpa)

            % calculate vehicle control messages
            obj.out_of_map_limits = false(obj.amount, 1);
            % time offset to account for slow start of MATLABs JIT
            % compilation
            time_offset = uint64(3 * 1e9);

            for vehicle_index = obj.vehicle_indices_controlled
                % simulate change of state
                i_vehicle = (obj.vehicle_indices_controlled == vehicle_index);

                obj.measurements(vehicle_index) = PlantMeasurement( ...
                    info.y_predicted(1, 1, i_vehicle), ... % x
                    info.y_predicted(2, 1, i_vehicle), ... % y
                    info.y_predicted(3, 1, i_vehicle), ... % yaw
                    mpa.trims(info.predicted_trims(i_vehicle, 1)).speed, ...
                    mpa.trims(info.predicted_trims(i_vehicle, 1)).steering ...
                );

                trajectory_points(1:obj.Hp) = TrajectoryPoint;

                for i = 1:obj.Hp
                    trajectory_points(i).t.nanoseconds = ...
                        uint64( ...
                        obj.time_now ...
                        + i_traj_pt * obj.dt_period_nanos ...
                        + time_offset ...
                    );
                    trajectory_points(i).px = info.y_predicted(1, i, vehicle_index);
                    trajectory_points(i).py = info.y_predicted(2, i, vehicle_index);

                    yaw = info.y_predicted(3, i, vehicle_index);

                    speed = mpa.trims(info.predicted_trims(i_vehicle, i)).speed;

                    trajectory_points(i).vx = cos(yaw) * speed;
                    trajectory_points(i).vy = sin(yaw) * speed;
                end

                obj.out_of_map_limits(vehicle_index) = obj.is_veh_at_map_border(trajectory_points);

                vehicle_command_trajectory = VehicleCommandTrajectory;
                vehicle_command_trajectory.vehicle_id = uint8(obj.all_vehicle_ids(vehicle_index));
                vehicle_command_trajectory.trajectory_points = trajectory_points;
                vehicle_command_trajectory.header.create_stamp.nanoseconds = ...
                    uint64(posixtime(datetime('now')));

                vehicle_command_trajectory.header.valid_after_stamp.nanoseconds = ...
                    uint64( ...
                    obj.time_now ...
                    + obj.dt_period_nanos ...
                    + time_offset ...
                    + 1 ... % to guarantee interpolating is possible
                );

                obj.writer_vehicleCommandTrajectory.write(vehicle_command_trajectory);
            end

        end

        function result = should_stop(obj)
            [~, result] = read_system_trigger(obj.reader_systemTrigger, obj.trigger_stop);

            if any(obj.out_of_map_limits)
                result = true;
            end

        end

        function end_run(~)
            disp('End')
        end

    end

    methods (Access = private)
        % helper function
        function stop_experiment = is_veh_at_map_border(~, trajectory_points)
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

        function prepare_dds(obj)
            % getenv('HOME'), 'dev/software/high_level_controller/examples/matlab' ...
            common_cpm_functions_path = fullfile( ...
                getenv('HOME'), 'dev/software/high_level_controller/examples/matlab' ...
            );
            assert(isfolder(common_cpm_functions_path), 'Missing folder "%s".', common_cpm_functions_path);
            addpath(common_cpm_functions_path);

            matlabDomainId = 1;
            [obj.dds_participant, obj.reader_vehicleStateList, obj.writer_vehicleCommandTrajectory, ~, obj.reader_systemTrigger, obj.writer_readyStatus, obj.trigger_stop, obj.writer_vehicleCommandDirect] = init_script(matlabDomainId); % #ok<ASGLU>

            % Set reader properties
            obj.reader_vehicleStateList.WaitSet = true;
            obj.reader_vehicleStateList.WaitSetTimeout = 5; % [s]
        end

    end

end
