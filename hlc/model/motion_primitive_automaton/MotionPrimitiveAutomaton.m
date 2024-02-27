classdef MotionPrimitiveAutomaton
    % MOTIONPRIMITVEAUTOMATON   MotionPrimitiveAutomaton

    properties
        maneuvers % cell(n_trims, n_trims)
        successor_trims % cell(n_trims, Hp), the successor trims of each trim
        trims % A struct array of the specified trim_inputs
        transition_matrix_single % Matrix (nTrims x nTrims x horizon_length)
        trim_tuple % Matrix with trim indices ((nTrims1*nTrims2*...) x nVehicles)
        transition_matrix_mean_speed % nTrims-by-nTrims matrix, each entry is the mean speed of the two connected trims
        distance_to_equilibrium uint16 % Distance in graph from current state to equilibrium state (nTrims x 1)
        recursive_feasibility
        local_reachable_sets % local reachable sets of each trim (possibly non-convex)
        local_reachable_sets_conv; % Convexified local reachable sets of each trim
        local_center_trajectory % local trajcetory of the center point
        local_reachable_sets_CP % local reachable sets of the center point
        trims_stop % trims with zero speed
    end

    properties (Access = protected)
    end

    methods

        function obj = MotionPrimitiveAutomaton(options, model)

            arguments
                options (1, 1) Config
                model (1, 1) VehicleModel = BicycleModel(Vehicle().Lf, Vehicle().Lr)
            end

            % Constructor
            % trim_inputs is a matrix of size (nTrims x nu)
            % trim_adjacency is a matrix of size (nTrims x nTrims),
            %   read as: rows are start trims and columns are end trims
            % N is the horizon length

            max_acceleration_m_s2 = 0.64;
            max_deceleration_m_s2 = 0.64;
            max_acceleration_per_dt = max_acceleration_m_s2 * options.dt_seconds;
            max_deceleration_per_dt = max_deceleration_m_s2 * options.dt_seconds;

            if options.is_prioritized
                nVeh_mpa = 1;
            else
                nVeh_mpa = options.amount;
            end

            % path of the MPA
            [file_path, ~, ~] = fileparts(mfilename('fullpath'));
            folder_target = [file_path, filesep, 'library'];

            if ~isfolder(folder_target)
                % create target folder if not exist
                mkdir(folder_target)
            end

            % for example: MPA_trims12_Hp6, MPA_trims12_Hp6_parl_non-convex
            mpa_instance_name = FileNameConstructor.get_mpa_name(options);

            mpa_full_path = fullfile(folder_target, mpa_instance_name);

            % if the needed MPA is already exist in the library, simply load
            % it, otherwise it will be built and saved to the library.
            % Note: if MPA properties are changed, then rebuild all MPAs!

            if isfile(mpa_full_path)

                try
                    fprintf('Loading MPA... ');
                    load(mpa_full_path, "mpa");
                    obj = mpa;
                    fprintf('done.\n')
                    return
                catch
                    fprintf('could not load, building new MPA.\n')
                end

            end

            obj.recursive_feasibility = options.recursive_feasibility;

            [trim_inputs, trim_adjacency] = choose_trims(options.mpa_type, max_acceleration_per_dt, max_deceleration_per_dt);
            n_trims = length(trim_inputs);

            obj.transition_matrix_single = zeros([size(trim_adjacency), options.Hp]);
            obj.transition_matrix_single(:, :, :) = repmat(trim_adjacency, 1, 1, options.Hp);

            obj.transition_matrix_mean_speed = zeros(size(trim_adjacency));

            for iTrim = 1:length(trim_inputs)

                for jTrim = 1:length(trim_inputs)

                    if trim_adjacency(iTrim, jTrim) ~= 0
                        obj.transition_matrix_mean_speed(iTrim, jTrim) = (trim_inputs(iTrim, 2) + trim_inputs(jTrim, 2)) / 2;
                    end

                end

            end

            % trim struct array
            % BicycleModel
            if model.nu == 2
                obj.trims = struct('steering', 0, 'speed', 0);
                % BicycleModelConstSpeed
            elseif model.nu == 1
                obj.trims = struct('steering', 0);
            end

            for i = 1:n_trims
                obj.trims(i) = generate_trim(model, trim_inputs(i, :));
            end

            % trims with zero speed
            obj.trims_stop = find([obj.trims.speed] == 0);
            % maneuver cell/struct matrix
            for i = 1:n_trims

                for j = 1:n_trims

                    if obj.transition_matrix_single(i, j, 1)
                        obj.maneuvers{i, j} = generate_maneuver(model, obj.trims(i), obj.trims(j), options);
                        % transform maneuver area to polyshape
                        obj.maneuvers{i, j}.areaPoly = polyshape(obj.maneuvers{i, j}.area(1, :), obj.maneuvers{i, j}.area(2, :), Simplify = false);
                    end

                end

            end

            % compute distance to equilibrium state
            eq_states = find(trim_inputs(:, 2) == 0);
            adj_trims = graph(obj.transition_matrix_single(:, :, 1));
            obj.distance_to_equilibrium = min(distances(adj_trims, eq_states), [], 1);

            % compute trim tuple (vertices)
            trim_index_list = cell(nVeh_mpa, 1);
            [trim_index_list{:}] = deal(1:n_trims);
            obj.trim_tuple = cartprod(trim_index_list{:});

            if options.recursive_feasibility
                obj.transition_matrix_single = compute_time_varying_transition_matrix(obj);
            end

            for i_step = 1:size(obj.transition_matrix_single, 3)

                for i_trim = 1:size(obj.transition_matrix_single, 1)
                    obj.successor_trims{i_trim, i_step} = find(obj.transition_matrix_single(i_trim, :, i_step));
                end

            end

            % variables to store reachable sets in different time steps
            obj.local_reachable_sets = cell(n_trims, options.Hp);
            obj.local_reachable_sets_conv = cell(n_trims, options.Hp);

            % For parallel computation, reachability analysis are used
            % no need for reachability analysis if only one vehicle
            is_calculate_reachable_sets_of_CP = false; % whether to calculate center point's reachable sets

            if options.is_use_dynamic_programming
                % use dynamic programming
                [obj.local_reachable_sets, obj.local_reachable_sets_conv, obj.local_center_trajectory, obj.local_reachable_sets_CP, ~] = ...
                    reachability_analysis_offline_DP(obj, options.Hp, is_calculate_reachable_sets_of_CP);
            else
                % otherwise use brute-force algorithm
                [obj.local_reachable_sets, obj.local_reachable_sets_conv, obj.local_center_trajectory, obj.local_reachable_sets_CP, ~] = ...
                    reachability_analysis_offline(obj, options.Hp, is_calculate_reachable_sets_of_CP);
            end

            if options.computation_mode ~= ComputationMode.parallel_threads
                % save mpa to library
                % If computing in parallel on one machine, this causes file
                % access conflicts.
                save_mpa(obj, mpa_full_path);
            end

        end

        function max_speed = get_max_speed_of_mpa(obj)
            % returns maximum speed of mpa (1 x 1)
            max_speed = max([obj.trims(:).speed]);
        end

        function straight_speeds = get_straight_speeds_of_mpa(obj)
            % return non-negative speeds of trims with steering=0
            non_negative_speed_trims = find([obj.trims(:).speed] > 0 & [obj.trims(:).steering] == 0);
            straight_speeds = [obj.trims(non_negative_speed_trims).speed];
        end

        function trim_index = trim_from_values(obj, speed, steering)
            % get closest trim based on speed and steering values
            %
            % the function uses a 2D norm to find the closest trim
            % therefore speed and steering are normalized to a range of [0, 1]

            trims_speed = [obj.trims.speed];
            trims_steering = [obj.trims.steering];

            % if the steering is 0 no different steering should be associated
            if steering == 0
                % get the trim indices that have a steering of 0
                indices_no_steering = find(trims_steering == 0);
                % calculate the speed distances to each trim
                speed_distances = abs(trims_speed - speed);
                % get the index of the trim with the minimum speed distances
                [~, index_min_distance] = min(speed_distances(indices_no_steering));
                % return the trim_index without steering and the lowest speed distance
                trim_index = indices_no_steering(index_min_distance);
                return
            end

            % get center and scale to normalize speed
            trims_speed_center = min(trims_speed);
            trims_speed_scale = max(trims_speed) - min(trims_speed);

            % get center and scale to normalize steering
            trims_steering_center = min(trims_steering);
            trims_steering_scale = max(trims_steering) - min(trims_steering);

            % normalize trims speed and steering
            trims_speed_norm = (trims_speed - trims_speed_center) / trims_speed_scale;
            trims_steering_norm = (trims_steering - trims_steering_center) / trims_steering_scale;

            % normalize input speed and steering to the trims ranges
            speed_norm = (speed - trims_speed_center) / trims_speed_scale;
            steering_norm = (steering - trims_steering_center) / trims_steering_scale;

            % calculate the distance to each trim
            trims_distances = vecnorm([trims_speed_norm - speed_norm; trims_steering_norm - steering_norm], 2, 1);

            % find closest trim by the minimum distance
            [~, trim_index] = min(trims_distances);
        end

        function transition_matrix_single = compute_time_varying_transition_matrix(obj)
            N = size(obj.transition_matrix_single, 3);
            transition_matrix_single = obj.transition_matrix_single;

            for k = 1:N
                % Columns are end trims. Forbid trims whose distance
                % to an equilbrium state is too high
                % (equilibrium state must be reachable within Hp-k steps)
                k_to_go = N - k;
                transition_matrix_single(:, obj.distance_to_equilibrium > k_to_go, k) = 0;
            end

        end

        function [reachable_sets_local, reachable_sets_conv_local, center_trajectory, reachable_sets_CP, trimsInfo] = reachability_analysis_offline(obj, Hp, is_calculate_reachable_sets_of_CP)
            % Calculate local reachable sets starting from a certain trim,
            % which can be used for online reachability analysis
            %
            % INPUT:
            %   obj: motion primitive automaton calss
            %
            %   Hp: prediction horizon
            %
            %   is_calculate_reachable_sets_of_CP: whether to calculate center point's reachable sets
            %
            % OUTPUT:
            %   reachable_sets_local: cell [n_trims x Hp]. The union of local reachable
            %   sets
            %
            %   reachable_sets_conv_local: cell [n_trims x Hp]. The convexified union
            %   of local reachable sets

            offline_computation_start = tic;
            threshold_Hp = 6;

            if Hp > threshold_Hp
                disp(['Computing reachable sets now...' newline ...
                          'Since the prediction horizon is ' num2str(Hp) ' (more than ' num2str(threshold_Hp) '), it may take several minutes.'])
            else
                disp('Computing local offline reachable sets now...')
            end

            disp('Note this only needs to be done once as later they will be saved to motion-primitive-automaton library.')

            n_trims = numel(obj.trims);
            reachable_sets_local = repmat({polyshape}, n_trims, Hp); % cell array with empty polyshapes
            reachable_sets_conv_local = cell(n_trims, Hp);
            center_trajectory = cell(n_trims, Hp);
            reachable_sets_CP = repmat({polyshape}, n_trims, Hp);

            trimsInfo = struct;

            % display progress
            textprogressbar('Computing local offline reachable sets: ');
            progress = 0;
            textprogressbar(progress);

            for i = 1:n_trims

                for t = 1:Hp

                    if t == 1 % root trim
                        trimsInfo(i, t).parentTrims = i;
                    else % The child trims become parent trims of the next time step
                        trimsInfo(i, t).parentTrims = trimsInfo(i, t - 1).childTrims;
                    end

                    % variable to store all child trims of the parent trims
                    trimsInfo(i, t).childTrims = [];
                    trimsInfo(i, t).childNum = [];
                    % find child trims of the parent trims
                    for i_Trim = 1:length(trimsInfo(i, t).parentTrims)
                        find_child = find(obj.transition_matrix_single(trimsInfo(i, t).parentTrims(i_Trim), :, t));
                        trimsInfo(i, t).childTrims = [trimsInfo(i, t).childTrims find_child];
                        trimsInfo(i, t).childNum = [trimsInfo(i, t).childNum length(find_child)];
                    end

                    % loop through all parent trims
                    for j = 1:length(trimsInfo(i, t).parentTrims)

                        if t == 1
                            x0 = 0;
                            y0 = 0;
                            yaw0 = 0;
                        else
                            x0 = trimsInfo(i, t - 1).maneuvers{j}.xs(end);
                            y0 = trimsInfo(i, t - 1).maneuvers{j}.ys(end);
                            yaw0 = trimsInfo(i, t - 1).maneuvers{j}.yaws(end);
                        end

                        % loop through all child trims
                        for k = 1:trimsInfo(i, t).childNum(j)
                            trim_start = trimsInfo(i, t).parentTrims(j);
                            child_ordinal = sum(trimsInfo(i, t).childNum(1:j - 1)) + k;
                            trim_end = trimsInfo(i, t).childTrims(child_ordinal);

                            % tranlates the local coordinates to global coordinates
                            [trimsInfo(i, t).maneuvers{child_ordinal}.xs, trimsInfo(i, t).maneuvers{child_ordinal}.ys] = ...
                                translate_global(yaw0, x0, y0, obj.maneuvers{trim_start, trim_end}.xs, obj.maneuvers{trim_start, trim_end}.ys);
                            trimsInfo(i, t).maneuvers{child_ordinal}.yaws = yaw0 + obj.maneuvers{trim_start, trim_end}.yaws;

                            % occupied area of the translated maneuvers
                            [area_x, area_y] = ...
                                translate_global(yaw0, x0, y0, obj.maneuvers{trim_start, trim_end}.area(1, :), obj.maneuvers{trim_start, trim_end}.area(2, :));
                            trimsInfo(i, t).maneuvers{child_ordinal}.area = [area_x; area_y];
                            trimsInfo(i, t).maneuvers{child_ordinal}.areaPoly = polyshape(area_x, area_y, Simplify = false);
                        end

                    end

                    % MATLAB function `union()` can union multiple polygons
                    % at the same time, but the number should not be very large
                    % as it will run significantly slow.
                    size_union = 15;
                    n_union_times = ceil(length(trimsInfo(i, t).maneuvers) / size_union);

                    for p = 1:n_union_times

                        if p == n_union_times
                            areaPolys = cellfun(@(c) [c.areaPoly], trimsInfo(i, t).maneuvers((p - 1) * size_union + 1:end));
                        else
                            areaPolys = cellfun(@(c) [c.areaPoly], trimsInfo(i, t).maneuvers((p - 1) * size_union + 1:p * size_union));
                        end

                        reachable_sets_local{i, t} = union([areaPolys, reachable_sets_local{i, t}]);
                    end

                    reachable_sets_conv_local{i, t} = convhull(reachable_sets_local{i, t}); % convexify

                    if is_calculate_reachable_sets_of_CP
                        % calculate trajectory and convexified reachable sets of the center point
                        center_trajectory{i, t} = cellfun(@(c) [c.xs; c.ys], trimsInfo(i, t).maneuvers, UniformOutput = false);
                        center_tra_xy = [center_trajectory{i, t}{:}];
                        idx_center_tra_conv = convhull(center_tra_xy(1, :), center_tra_xy(2, :));
                        reachable_sets_CP{i, t} = polyshape(center_tra_xy(1, idx_center_tra_conv), center_tra_xy(2, idx_center_tra_conv));
                    end

                    % display progress
                    progress = ((i - 1) * Hp + t) / (n_trims * Hp) * 100; % percentage
                    textprogressbar(progress);
                end

            end

            duration_computation = toc(offline_computation_start);
            textprogressbar('done');
            disp(['Finished in ' num2str(duration_computation) ' seconds.'])
        end

        function save_mpa(obj, mpa_full_path)
            % Save MPA to library
            mpa = obj;
            save(mpa_full_path, 'mpa', '-v7.3');
        end

        function [reachable_sets_local, reachable_sets_conv_local, center_trajectory, reachable_sets_CP, trimsInfo] = reachability_analysis_offline_DP(obj, Hp, is_calculate_reachable_sets_of_CP)
            % Calculate local reachable sets starting from a certain trim using dynamic programming,
            % which can be used for online reachability analysis
            %
            % INPUT:
            %   obj: motion primitive automaton calss
            %
            %   Hp: prediction horizon
            %
            %   is_calculate_reachable_sets_of_CP: whether to calculate center point's reachable sets
            %
            % OUTPUT:
            %   reachable_sets_local: cell [n_trims x Hp]. The union of local reachable
            %   sets
            %
            %   reachable_sets_conv_local: cell [n_trims x Hp]. The convexified union
            %   of local reachable sets

            offline_computation_start = tic;
            threshold_Hp = 10;

            if Hp > threshold_Hp
                fprintf(['Computing reachable sets now...' newline ...
                             'Since the prediction horizon is ' num2str(Hp) ' (more than ' num2str(threshold_Hp) '), it may take several minutes.'])
            else
                fprintf('Computing local offline reachable sets now...')
            end

            Hp_half = ceil(Hp / 2); % for time step greater then half of the prediction horizon, dynamic programming is used to save computation time

            n_trims = numel(obj.trims);
            reachable_sets_local = repmat({polyshape}, n_trims, Hp); % cell array with empty polyshapes
            reachable_sets_local_HpHalf = repmat({polyshape}, n_trims, 1); % reachable sets of the time step "Hp_half" assuming that "Hp_half" is the last time step (so that the last trim must end with the equilibrium trim)
            reachable_sets_conv_local = cell(n_trims, Hp);
            reachable_sets_conv_local_HpHalf = cell(n_trims, 1);
            center_trajectory = cell(n_trims, Hp);
            reachable_sets_CP = repmat({polyshape}, n_trims, Hp);

            trimsInfo = struct;
            trimsInfoHpHalf = struct;

            % display progress
            textprogressbar('Computing local offline reachable sets: ');
            progress = 0;
            textprogressbar(progress);

            for i = 1:n_trims

                for t = 1:Hp_half

                    if t == 1 % root trim
                        trimsInfo(i, t).parentTrims = i;
                    else % The child trims become parent trims of the next time step
                        trimsInfo(i, t).parentTrims = trimsInfo(i, t - 1).childTrims;
                    end

                    % variable to store all child trims of the parent trims
                    trimsInfo(i, t).childTrims = [];
                    trimsInfo(i, t).childNum = [];
                    % find child trims of the parent trims
                    for i_Trim = 1:length(trimsInfo(i, t).parentTrims)
                        find_child = find(obj.transition_matrix_single(trimsInfo(i, t).parentTrims(i_Trim), :, t));
                        trimsInfo(i, t).childTrims = [trimsInfo(i, t).childTrims find_child];
                        trimsInfo(i, t).childNum = [trimsInfo(i, t).childNum length(find_child)];
                    end

                    % loop through all parent trims
                    for j = 1:length(trimsInfo(i, t).parentTrims)

                        if t == 1
                            x0 = 0;
                            y0 = 0;
                            yaw0 = 0;
                        else
                            x0 = trimsInfo(i, t - 1).maneuvers{j}.xs(end);
                            y0 = trimsInfo(i, t - 1).maneuvers{j}.ys(end);
                            yaw0 = trimsInfo(i, t - 1).maneuvers{j}.yaws(end);
                        end

                        % loop through all child trims
                        for k = 1:trimsInfo(i, t).childNum(j)
                            trim_start = trimsInfo(i, t).parentTrims(j);
                            child_ordinal = sum(trimsInfo(i, t).childNum(1:j - 1)) + k;
                            trim_end = trimsInfo(i, t).childTrims(child_ordinal);

                            % tranlates the local coordinates to global coordinates
                            [trimsInfo(i, t).maneuvers{child_ordinal}.xs, trimsInfo(i, t).maneuvers{child_ordinal}.ys] = ...
                                translate_global(yaw0, x0, y0, obj.maneuvers{trim_start, trim_end}.xs, obj.maneuvers{trim_start, trim_end}.ys);
                            trimsInfo(i, t).maneuvers{child_ordinal}.yaws = yaw0 + obj.maneuvers{trim_start, trim_end}.yaws;

                            % occupied area of the translated maneuvers
                            [area_x, area_y] = ...
                                translate_global(yaw0, x0, y0, obj.maneuvers{trim_start, trim_end}.area(1, :), obj.maneuvers{trim_start, trim_end}.area(2, :));
                            trimsInfo(i, t).maneuvers{child_ordinal}.area = [area_x; area_y];
                            trimsInfo(i, t).maneuvers{child_ordinal}.areaPoly = polyshape(area_x, area_y, Simplify = false);
                        end

                    end

                    % MATLAB function `union()` can union multiple polygons
                    % at the same time, but the number should not be very large
                    % as it will run significantly slow.
                    size_union = 15;
                    n_union_times = ceil(length(trimsInfo(i, t).maneuvers) / size_union);

                    for p = 1:n_union_times

                        if p == n_union_times
                            areaPolys_tmp = cellfun(@(c) [c.areaPoly], trimsInfo(i, t).maneuvers((p - 1) * size_union + 1:end));
                        else
                            areaPolys_tmp = cellfun(@(c) [c.areaPoly], trimsInfo(i, t).maneuvers((p - 1) * size_union + 1:p * size_union));
                        end

                        reachable_sets_local{i, t} = union([areaPolys_tmp, reachable_sets_local{i, t}]);
                    end

                    reachable_sets_conv_local{i, t} = convhull(reachable_sets_local{i, t}); % convexify

                    if is_calculate_reachable_sets_of_CP
                        % calculate trajectory and convexified reachable sets of the center point
                        center_trajectory{i, t} = cellfun(@(c) [c.xs; c.ys], trimsInfo(i, t).maneuvers, UniformOutput = false);
                        center_tra_xy = [center_trajectory{i, t}{:}];
                        idx_center_tra_conv = convhull(center_tra_xy(1, :), center_tra_xy(2, :));
                        reachable_sets_CP{i, t} = polyshape(center_tra_xy(1, idx_center_tra_conv), center_tra_xy(2, idx_center_tra_conv));
                    end

                    % display progress
                    progress = ((i - 1) * Hp + t) / (n_trims * Hp) * 100/2; % percentage. Divide by 2 since the left half will be calculated using dynamic programming later
                    textprogressbar(progress);
                end

            end

            % use dynamic programming to finished the remaining half
            % pridition horizon

            % assume that Hp_half is the final time step and thus
            % the child trim must end with the equilibrium trim
            if Hp > 1

                for ii = 1:n_trims
                    tt = Hp_half;

                    % parent trims are unchanged, only child trims are reducted
                    trimsInfoHpHalf(ii, 1).parentTrims = trimsInfo(ii, tt).parentTrims;
                    % variable to store all child trims of the parent trims
                    trimsInfoHpHalf(ii, 1).childTrims = [];
                    trimsInfoHpHalf(ii, 1).childNum = [];
                    % find child trims of the parent trims
                    for i_Trim = 1:length(trimsInfoHpHalf(ii, 1).parentTrims)
                        find_child = find(obj.transition_matrix_single(trimsInfoHpHalf(ii, 1).parentTrims(i_Trim), :, end));
                        trimsInfoHpHalf(ii, 1).childTrims = [trimsInfoHpHalf(ii, 1).childTrims find_child];
                        trimsInfoHpHalf(ii, 1).childNum = [trimsInfoHpHalf(ii, 1).childNum length(find_child)];
                    end

                    % loop through all parent trims
                    for j = 1:length(trimsInfoHpHalf(ii, 1).parentTrims)

                        if tt == 1
                            x0 = 0;
                            y0 = 0;
                            yaw0 = 0;
                        else
                            x0 = trimsInfo(ii, tt - 1).maneuvers{j}.xs(end);
                            y0 = trimsInfo(ii, tt - 1).maneuvers{j}.ys(end);
                            yaw0 = trimsInfo(ii, tt - 1).maneuvers{j}.yaws(end);
                        end

                        % loop through all child trims
                        for k = 1:trimsInfoHpHalf(ii, 1).childNum(j)
                            trim_start = trimsInfoHpHalf(ii, 1).parentTrims(j);
                            child_ordinal = sum(trimsInfoHpHalf(ii, 1).childNum(1:j - 1)) + k;
                            trim_end = trimsInfoHpHalf(ii, 1).childTrims(child_ordinal);

                            % tranlates the local coordinates to global coordinates
                            [trimsInfoHpHalf(ii, 1).maneuvers{child_ordinal}.xs, trimsInfoHpHalf(ii, 1).maneuvers{child_ordinal}.ys] = ...
                                translate_global(yaw0, x0, y0, obj.maneuvers{trim_start, trim_end}.xs, obj.maneuvers{trim_start, trim_end}.ys);
                            trimsInfoHpHalf(ii, 1).maneuvers{child_ordinal}.yaws = yaw0 + obj.maneuvers{trim_start, trim_end}.yaws;

                            % occupied area of the translated maneuvers
                            [area_x, area_y] = ...
                                translate_global(yaw0, x0, y0, obj.maneuvers{trim_start, trim_end}.area(1, :), obj.maneuvers{trim_start, trim_end}.area(2, :));
                            trimsInfoHpHalf(ii, 1).maneuvers{child_ordinal}.area = [area_x; area_y];
                            trimsInfoHpHalf(ii, 1).maneuvers{child_ordinal}.areaPoly = polyshape(area_x, area_y, Simplify = false);
                        end

                    end

                    n_union_times = ceil(length(trimsInfoHpHalf(ii, 1).maneuvers) / size_union);

                    for p = 1:n_union_times

                        if p == n_union_times
                            areaPolys_tmp = cellfun(@(c) [c.areaPoly], trimsInfoHpHalf(ii, 1).maneuvers((p - 1) * size_union + 1:end));
                        else
                            areaPolys_tmp = cellfun(@(c) [c.areaPoly], trimsInfoHpHalf(ii, 1).maneuvers((p - 1) * size_union + 1:p * size_union));
                        end

                        reachable_sets_local_HpHalf{ii, 1} = union([areaPolys_tmp, reachable_sets_local_HpHalf{ii, 1}]);
                    end

                    reachable_sets_conv_local_HpHalf{ii, 1} = convhull(reachable_sets_local_HpHalf{ii, 1}); % convexify
                end

            end

            for i = 1:n_trims

                for t = Hp_half + 1:Hp
                    trimInfo = trimsInfo(i, t - Hp_half);
                    areaPolys = repmat(polyshape, 1, length(trimInfo.maneuvers));

                    for j = 1:length(trimInfo.maneuvers)
                        parentTrim = trimInfo.childTrims(j);

                        if t == Hp
                            [area_x, area_y] = translate_global(trimInfo.maneuvers{j}.yaws(end), trimInfo.maneuvers{j}.xs(end), trimInfo.maneuvers{j}.ys(end), ...
                                reachable_sets_local_HpHalf{parentTrim, 1}.Vertices(:, 1)', reachable_sets_local_HpHalf{parentTrim, 1}.Vertices(:, 2)');
                        else
                            [area_x, area_y] = translate_global(trimInfo.maneuvers{j}.yaws(end), trimInfo.maneuvers{j}.xs(end), trimInfo.maneuvers{j}.ys(end), ...
                                reachable_sets_local{parentTrim, Hp_half}.Vertices(:, 1)', reachable_sets_local{parentTrim, Hp_half}.Vertices(:, 2)');
                        end

                        areaPolys(j) = polyshape(area_x, area_y, Simplify = false);
                    end

                    % union polyshapes
                    n_union_times = ceil(length(areaPolys) / size_union);

                    for p = 1:n_union_times

                        if p == n_union_times
                            areaPolys_tmp = areaPolys((p - 1) * size_union + 1:end);
                        else
                            areaPolys_tmp = areaPolys((p - 1) * size_union + 1:p * size_union);
                        end

                        reachable_sets_local{i, t} = union([areaPolys_tmp, reachable_sets_local{i, t}]);
                    end

                    reachable_sets_conv_local{i, t} = convhull(reachable_sets_local{i, t}); % convexify

                    % display progress
                    progress = 50 + ((i - 1) * Hp + t) / (n_trims * Hp) * 100/2; % percentage
                    textprogressbar(progress);
                end

            end

            duration_computation = toc(offline_computation_start);
            textprogressbar('done');
            fprintf('done (%.2f s).\n', duration_computation);

        end

        function transformed_reachable_sets = reachable_sets_at_pose(obj, x, y, yaw, trim)
            % the function takes the local reachable sets for the current trim
            % and translates it to the current position (x, y) and yaw
            %
            % Output:
            %   transformed_reachable_sets (1, Hp) cell of polyshape objects

            arguments
                obj MotionPrimitiveAutomaton
                x (1, 1) double % current x coordinate
                y (1, 1) double % current y coordinate
                yaw (1, 1) double % current yaw
                trim (1, 1) double % current trim
            end

            local_reachable_sets_trim = obj.local_reachable_sets_conv(trim, :);

            Hp = size(local_reachable_sets_trim, 2);
            transformed_reachable_sets = cell(1, Hp);

            % get the full reachable sets in global frame as polyshape
            for t = 1:Hp
                % translate the local reachable sets to global coordinates
                [reachable_set_x, reachable_set_y] = translate_global( ...
                    yaw, ...
                    x, ...
                    y, ...
                    local_reachable_sets_trim{t}.Vertices(:, 1)', ...
                    local_reachable_sets_trim{t}.Vertices(:, 2)' ...
                );

                transformed_reachable_sets{t} = polyshape( ...
                    reachable_set_x, ...
                    reachable_set_y, ...
                    Simplify = false ...
                );
            end

        end

        function result = maximum_branching_factor(obj)
            % Returns the maximum branching factor of the motion primitive automaton
            result = max(sum(obj.transition_matrix_single, 2), [], 'all');
        end

        function plot(obj, optional)

            arguments
                obj (1, 1) MotionPrimitiveAutomaton;
                optional.y_lim (1, 2) double = [-0.1, 1.0];
                optional.x_lim (1, 2) double = rad2deg(pi / 5 * [-1, 1]);
                optional.k (1, 1) double = 1;
                optional.fig (1, 1) matlab.ui.Figure = figure(Visible = "on");
                optional.with_labels (1, 1) logical = true;
            end

            set(0, "CurrentFigure", optional.fig);

            trim_inputs = obj.trims;

            trim_adjacency = obj.transition_matrix_single(:, :, optional.k);

            angle = rad2deg([trim_inputs.steering]);
            speed = [trim_inputs.speed];
            G = digraph(trim_adjacency, 'omitSelfLoops');

            plot(G, 'XData', angle, 'YData', speed, ...
                ArrowSize = 5, ...
                MarkerSize = 3, ...
                NodeColor = rwth_color_order(1), ...
                EdgeColor = rwth_color_order_50(1), ...
                EdgeAlpha = 1 ...
            );

            if optional.with_labels
                xlabel('Steering Angle $\delta$ [$^{\circ}$]', Interpreter = 'latex');
                ylabel('Speed $\mathrm{v}$ [m/s]', Interpreter = 'latex');
            end

            if isfield(optional, 'x_lim')
                xlim(optional.x_lim);
            end

            if isfield(optional, 'y_lim')
                ylim(optional.y_lim);
            end

            grid on

        end

        function plot_over_time(obj, optional)

            arguments
                obj (1, 1) MotionPrimitiveAutomaton;
                optional.y_lim (1, 2) double = [-0.1, 1.1];
                optional.fig (1, 1) matlab.ui.Figure = figure(Visible = "on");
            end

            set(0, "CurrentFigure", optional.fig);

            Hp = size(obj.transition_matrix_single, 3);
            tiledLayoutHandle = tiledlayout( ...
                1, Hp, ...
                TileSpacing = 'compact', ...
                Padding = 'compact' ...
            );

            for k = 1:Hp
                nexttile
                obj.plot(fig = optional.fig, ...
                    k = k, ...
                    with_labels = false, ...
                    y_lim = optional.y_lim ...
                );
                title(sprintf("$t=k+%d$", k - 1), Interpreter = 'latex');
            end

            xlabel(tiledLayoutHandle, 'Steering Angle $\delta$ [$^{\circ}$]', ...
                Interpreter = 'latex' ...
            );
            ylabel(tiledLayoutHandle, 'Speed $\mathrm{v}$ [m/s]', ...
                Interpreter = 'latex' ...
            );

        end

        function plot_local_reachable_sets(obj, optional)
            % PLOT_LOCAL_REACHABLE_SETS Visualize the reachable sets starting from
            % different root trims.

            arguments
                obj (1, 1) MotionPrimitiveAutomaton;
                optional.fig (1, 1) matlab.ui.Figure = figure(Name = "ReachableSets", Visible = "on");
            end

            set(0, "CurrentFigure", optional.fig);

            n_trims = size(obj.local_reachable_sets, 1);
            Hp = size(obj.local_reachable_sets, 2);

            t_fig = tiledlayout(n_trims, Hp, TileSpacing = 'compact');

            for i = 1:n_trims

                for t = 1:Hp
                    nexttile;
                    plot(obj.local_reachable_sets{i, t})
                    hold on
                    plot(obj.local_reachable_sets_conv{i, t})

                    if t == 1
                        ylabel(['root trim ', num2str(i)])
                    end

                    axis square
                    grid on
                    xlim([-0.2, 1.5]);
                    ylim([-1.4 1.4]);
                end

            end

            xlabel(t_fig, 'x [m]')
            ylabel(t_fig, 'y [m]')
            title(t_fig, 'Reachable Sets of Different Root Trims')
        end

    end

end
