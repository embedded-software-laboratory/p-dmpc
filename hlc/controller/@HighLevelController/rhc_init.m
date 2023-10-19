function rhc_init(obj, states_measured, trims_measured)
    % RHC_INIT  Preprocessing step for RHC controller

    % create indices struct only once for efficiency
    idx = indices();

    % calculate adjacency between CAVs and HDVs and HDV reachable sets
    for iHdv = 1:obj.scenario.options.manual_control_config.amount

        % determine HDV lanelet id based on HDV's position
        lanelet_struct = obj.scenario.road_raw_data.lanelet;
        state_hdv = states_measured(obj.scenario.options.amount + iHdv, :);
        lanelet_id_hdv = map_position_to_closest_lanelets( ...
            obj.scenario.lanelets, ...
            state_hdv(idx.x), ...
            state_hdv(idx.y) ...
        );

        % calculate the intersection of reachable sets with the current and
        % the successor lanelet (returned as cell array of polyshape objects
        % for each step in the prediction horizon)
        reachable_sets = obj.manual_vehicles(iHdv).compute_reachable_lane( ...
            state_hdv, ...
            lanelet_id_hdv ...
        );

        % determine empty polyshape objects
        empty_sets = cellfun(@(c) c.NumRegions == 0, reachable_sets);
        % fill empty reachable sets with a cell array containing an empty array
        obj.iter.hdv_reachable_sets(iHdv, empty_sets) = {[]};
        % convert polyshape in plain array (repeat first point to enclose the shape)
        obj.iter.hdv_reachable_sets(iHdv, ~empty_sets) = cellfun(@(c) ...
            [c.Vertices(:, 1)', c.Vertices(1, 1)'; c.Vertices(:, 2)', c.Vertices(1, 2)'], ...
            reachable_sets(~empty_sets), ...
            'UniformOutput', false ...
        );

        % update reduced coupling adjacency for cav/hdv-pairs
        for iVeh = obj.plant.indices_in_vehicle_list
            % determine CAV lanelet id based on CAV's position
            state_cav = states_measured(iVeh, :);
            % TODO isn't the lanelet_id_cav already available?
            lanelet_id_cav = map_position_to_closest_lanelets( ...
                obj.scenario.lanelets, ...
                state_cav(idx.x), ...
                state_cav(idx.y) ...
            );

            % note coupling if HDV is not behind CAV
            % if HDV is behind CAV and coupling is noted
            % the optimizer will probably not find a solution
            % since the CAV is totally in HDV's reachable set
            obj.iter.hdv_adjacency(iVeh, iHdv) = ~is_hdv_behind( ...
                lanelet_id_cav, ...
                state_cav, ...
                lanelet_id_hdv, ...
                state_hdv, ...
                lanelet_struct ...
            );
        end

    end

    % remove manual vehicle states for further calculations
    states_measured = states_measured(1:obj.scenario.options.amount, :);

    for iVeh = obj.plant.indices_in_vehicle_list
        % states of controlled vehicles can be measured directly
        obj.iter.x0(iVeh, :) = states_measured(iVeh, :);
        % get own trim
        obj.iter.trim_indices(iVeh) = trims_measured(iVeh);
        x0 = obj.iter.x0(iVeh, idx.x);
        y0 = obj.iter.x0(iVeh, idx.y);
        yaw0 = obj.iter.x0(iVeh, idx.heading);
        trim_current = obj.iter.trim_indices(iVeh);

        % Compute the predicted lanelets of iVeh vehicle
        % Byproducts: reference path and reference speed profile
        [predicted_lanelets, reference, v_ref] = get_predicted_lanelets(obj.scenario, obj.mpa, obj.iter, iVeh, x0, y0);

        % For non manual vehicles: Update ref speed and ref trajectory
        % reference speed and path points
        obj.iter.v_ref(iVeh, :) = v_ref;

        % equidistant points on the reference trajectory.
        obj.iter.reference_trajectory_points(iVeh, :, :) = reference.ReferencePoints;
        obj.iter.reference_trajectory_index(iVeh, :, :) = reference.ReferenceIndex;

        obj.iter.last_trajectory_index(iVeh) = reference.ReferenceIndex(end);

        if (obj.scenario.options.scenario_type == ScenarioType.commonroad)

            obj.iter.predicted_lanelets{iVeh} = predicted_lanelets;

            % Calculate the predicted lanelet boundary of vehicle iVeh based on its predicted lanelets
            predicted_lanelet_boundary = get_lanelets_boundary(predicted_lanelets, obj.scenario.lanelet_boundary, obj.scenario.vehicles(iVeh).lanelets_index, obj.scenario.options.environment, obj.scenario.vehicles(iVeh).is_loop);
            obj.iter.predicted_lanelet_boundary(iVeh, :) = predicted_lanelet_boundary;

        end

        % Compute reachable sets for vehicle iVeh
        obj.iter.reachable_sets(iVeh, :) = get_reachable_sets( ...
            x0, ...
            y0, ...
            yaw0, ...
            obj.mpa.local_reachable_sets_conv(trim_current, :), ...
            obj.iter.predicted_lanelet_boundary(iVeh, :), ...
            obj.scenario.options ...
        );

        % get vehicles currently occupied area
        x_rec1 = [-1, -1, 1, 1, -1] * (obj.scenario.vehicles(iVeh).Length / 2 + obj.scenario.options.offset); % repeat the first entry to enclose the shape
        y_rec1 = [-1, 1, 1, -1, -1] * (obj.scenario.vehicles(iVeh).Width / 2 + obj.scenario.options.offset);
        % calculate displacement of model shape
        [x_rec2, y_rec2] = translate_global(yaw0, x0, y0, x_rec1, y_rec1);
        obj.iter.occupied_areas{iVeh}.normal_offset = [x_rec2; y_rec2];

        x_rec1_without_offset = [-1, -1, 1, 1, -1] * (obj.scenario.vehicles(iVeh).Length / 2); % repeat the first entry to enclose the shape
        y_rec1_without_offset = [-1, 1, 1, -1, -1] * (obj.scenario.vehicles(iVeh).Width / 2);
        [x_rec2_without_offset, y_rec2_without_offset] = translate_global(yaw0, x0, y0, x_rec1_without_offset, y_rec1_without_offset);
        obj.iter.occupied_areas{iVeh}.without_offset = [x_rec2_without_offset; y_rec2_without_offset];

        % Get vehicle's occupied area of emergency braking maneuver
        % with normal offset

        %% emergency maneuver for vehicle iVeh
        % emergency left maneuver (without offset)
        turn_left_area_without_offset = obj.mpa.emergency_maneuvers{trim_current}.left{1};
        [turn_left_area_without_offset_x, turn_left_area_without_offset_y] = translate_global(yaw0, x0, y0, turn_left_area_without_offset(1, :), turn_left_area_without_offset(2, :));
        obj.iter.emergency_maneuvers{iVeh}.left_area_without_offset = [turn_left_area_without_offset_x; turn_left_area_without_offset_y];
        % emergency right maneuver (without offset)
        turn_right_area_without_offset = obj.mpa.emergency_maneuvers{trim_current}.right{1};
        [turn_right_area_without_offset_x, turn_right_area_without_offset_y] = translate_global(yaw0, x0, y0, turn_right_area_without_offset(1, :), turn_right_area_without_offset(2, :));
        obj.iter.emergency_maneuvers{iVeh}.right_area_without_offset = [turn_right_area_without_offset_x; turn_right_area_without_offset_y];
        % emergency braking maneuver (without offset)
        braking_area_without_offset = obj.mpa.emergency_maneuvers{trim_current}.braking_without_offset;
        [turn_braking_area_without_offset_x, turn_braking_area_without_offset_y] = translate_global(yaw0, x0, y0, braking_area_without_offset(1, :), braking_area_without_offset(2, :));
        obj.iter.emergency_maneuvers{iVeh}.braking_area_without_offset = [turn_braking_area_without_offset_x; turn_braking_area_without_offset_y];
        % emergency braking maneuver (with normal offset)
        braking_area = obj.mpa.emergency_maneuvers{trim_current}.braking_with_offset;
        [turn_braking_area_x, turn_braking_area_y] = translate_global(yaw0, x0, y0, braking_area(1, :), braking_area(2, :));
        obj.iter.emergency_maneuvers{iVeh}.braking_area = [turn_braking_area_x; turn_braking_area_y];

        if obj.scenario.options.is_prioritized
            %% Send data to sync obj.iter for all vehicles (especially needed for priority assignment)
            obj.traffic_communication{iVeh}.send_message( ...
                obj.k, ...
                obj.iter.x0(iVeh, :), ...
                obj.iter.trim_indices(iVeh), ...
                obj.iter.predicted_lanelets{iVeh}, ...
                obj.iter.occupied_areas{iVeh}, ...
                obj.iter.reachable_sets(iVeh, :) ...
            );
        end

    end

    if obj.scenario.options.is_prioritized
        %% read messages from other vehicles (There shouldn't be any other vehicles if centralized)

        for jVeh = obj.plant.indices_in_vehicle_list
            % loop over vehicle that reads the message
            other_vehicles = setdiff(1:obj.scenario.options.amount, jVeh);

            for kVeh = other_vehicles
                % loop over vehicle from which the messages are read
                latest_msg_i = obj.traffic_communication{jVeh}.read_message( ...
                    obj.plant.all_vehicle_ids(kVeh), ...
                    obj.k, ...
                    true ...
                );
                obj.iter.x0(kVeh, :) = [latest_msg_i.current_pose.x, latest_msg_i.current_pose.y, latest_msg_i.current_pose.heading, latest_msg_i.current_pose.speed];
                obj.iter.trim_indices(kVeh) = latest_msg_i.current_trim_index;
                obj.iter.predicted_lanelets{kVeh} = latest_msg_i.predicted_lanelets';
                occupied_areas = latest_msg_i.occupied_areas;
                obj.iter.occupied_areas{kVeh}.normal_offset(1, :) = occupied_areas(1).x;
                obj.iter.occupied_areas{kVeh}.normal_offset(2, :) = occupied_areas(1).y;
                obj.iter.occupied_areas{kVeh}.without_offset(1, :) = occupied_areas(2).x;
                obj.iter.occupied_areas{kVeh}.without_offset(2, :) = occupied_areas(2).y;
                obj.iter.reachable_sets(kVeh, :) = (arrayfun(@(array) {polyshape(array.x, array.y)}, latest_msg_i.reachable_sets))';

                % Calculate the predicted lanelet boundary of vehicle kVeh based on its predicted lanelets
                % TODO is lanelets_index of other vehicles up to date?
                if (obj.scenario.options.scenario_type == ScenarioType.commonroad)
                    predicted_lanelet_boundary = get_lanelets_boundary(obj.iter.predicted_lanelets{kVeh}, obj.scenario.lanelet_boundary, obj.scenario.vehicles(kVeh).lanelets_index, obj.scenario.options.environment, obj.scenario.vehicles(kVeh).is_loop);
                    obj.iter.predicted_lanelet_boundary(kVeh, :) = predicted_lanelet_boundary;
                end

                x0 = obj.iter.x0(kVeh, idx.x);
                y0 = obj.iter.x0(kVeh, idx.y);
                yaw0 = obj.iter.x0(kVeh, idx.heading);
                trim_current = obj.iter.trim_indices(kVeh);

                %% emergency maneuver for vehicle kVeh
                % emergency left maneuver (without offset)
                turn_left_area_without_offset = obj.mpa.emergency_maneuvers{trim_current}.left{1};
                [turn_left_area_without_offset_x, turn_left_area_without_offset_y] = translate_global(yaw0, x0, y0, turn_left_area_without_offset(1, :), turn_left_area_without_offset(2, :));
                obj.iter.emergency_maneuvers{kVeh}.left_area_without_offset = [turn_left_area_without_offset_x; turn_left_area_without_offset_y];
                % emergency right maneuver (without offset)
                turn_right_area_without_offset = obj.mpa.emergency_maneuvers{trim_current}.right{1};
                [turn_right_area_without_offset_x, turn_right_area_without_offset_y] = translate_global(yaw0, x0, y0, turn_right_area_without_offset(1, :), turn_right_area_without_offset(2, :));
                obj.iter.emergency_maneuvers{kVeh}.right_area_without_offset = [turn_right_area_without_offset_x; turn_right_area_without_offset_y];
                % emergency braking maneuver (without offset)
                braking_area_without_offset = obj.mpa.emergency_maneuvers{trim_current}.braking_without_offset;
                [turn_braking_area_without_offset_x, turn_braking_area_without_offset_y] = translate_global(yaw0, x0, y0, braking_area_without_offset(1, :), braking_area_without_offset(2, :));
                obj.iter.emergency_maneuvers{kVeh}.braking_area_without_offset = [turn_braking_area_without_offset_x; turn_braking_area_without_offset_y];
                % emergency braking maneuver (with normal offset)
                braking_area = obj.mpa.emergency_maneuvers{trim_current}.braking_with_offset;
                [turn_braking_area_x, turn_braking_area_y] = translate_global(yaw0, x0, y0, braking_area(1, :), braking_area(2, :));
                obj.iter.emergency_maneuvers{kVeh}.braking_area = [turn_braking_area_x; turn_braking_area_y];
            end

        end

    end

end
