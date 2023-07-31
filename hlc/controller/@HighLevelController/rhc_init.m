function rhc_init(obj, x_measured, trims_measured)
    % RHC_INIT  Preprocessing step for RHC controller

    idx = indices();
    visualize_trajectory_index_lab = false;
    visualize_boundaries_lab = false;
    predicted_lanelet_boundary = {[]};

    % init HDV: compute current lanelet id and reachable sets intersected
    % with current & successor lane;
    hdv_amount = obj.scenario.options.manual_control_config.amount ...
        * (~(obj.scenario.options.environment == Environment.Simulation));

    for iHdv = 1:hdv_amount

        % init reachable sets of hdvs
        lanelet_struct = obj.scenario.road_raw_data.lanelet;
        x_hdv = x_measured(obj.scenario.options.amount + iHdv, :);
        lanelet_id_hdv = map_position_to_closest_lanelets(obj.scenario.lanelets, x_hdv(idx.x), x_hdv(idx.y));
        reachable_sets = obj.manual_vehicles(iHdv).compute_reachable_lane(x_hdv, lanelet_id_hdv);

        % turn polyshape to plain array (repeat the first row to enclosed the shape)
        polyshapes = [reachable_sets{:}];
        empty_sets = [polyshapes.NumRegions] == 0;
        reachable_sets_array = cellfun(@(c) {[c.Vertices(:, 1)', c.Vertices(1, 1)'; c.Vertices(:, 2)', c.Vertices(1, 2)']}, reachable_sets(~empty_sets));
        obj.iter.hdv_reachable_sets(iHdv, empty_sets) = {[]};
        obj.iter.hdv_reachable_sets(iHdv, ~empty_sets) = reachable_sets_array;

        % update reduced coupling adjacency for cav/hdv-pairs
        for iVeh = obj.plant.indices_in_vehicle_list
            x_cav = x_measured(iVeh, :);
            lanelet_id_cav = map_position_to_closest_lanelets(obj.scenario.lanelets, x_cav(idx.x), x_cav(idx.y));
            obj.iter.hdv_adjacency(iVeh, iHdv) = ~is_hdv_behind( ...
                lanelet_id_cav, ...
                x_cav, ...
                lanelet_id_hdv, ...
                x_hdv, ...
                lanelet_struct ...
            );
        end

    end

    % remove manual vehicle states for further calculations
    x_measured = x_measured(1:obj.scenario.options.amount, :);

    for iVeh = obj.plant.indices_in_vehicle_list
        % states of controlled vehicles can be measured directly
        obj.iter.x0(iVeh, :) = x_measured(iVeh, :);
        % get own trim
        obj.iter.trim_indices(iVeh) = trims_measured(iVeh);
        x0 = obj.iter.x0(iVeh, idx.x);
        y0 = obj.iter.x0(iVeh, idx.y);
        yaw0 = obj.iter.x0(iVeh, idx.heading);
        trim_current = obj.iter.trim_indices(iVeh);

        % Compute the predicted lanelets of iVeh vehicle
        % Byproducts: reference path and reference speed profile
        [predicted_lanelets, reference, v_ref] = get_predicted_lanelets(obj.scenario, obj.iter, iVeh, x0, y0);

        % For non manual vehicles: Update ref speed and ref trajectory
        % reference speed and path points
        obj.iter.v_ref(iVeh, :) = v_ref;

        % equidistant points on the reference trajectory.
        obj.iter.reference_trajectory_points(iVeh, :, :) = reference.ReferencePoints;
        obj.iter.reference_trajectory_index(iVeh, :, :) = reference.ReferenceIndex;

        obj.iter.last_trajectory_index(iVeh) = reference.ReferenceIndex(end);

        if (obj.scenario.options.scenario_type == ScenarioType.commonroad)

            % if random path was updated, include the last lane before updating, because the predicted lane are planned starting from the updated lane
            if obj.iter.lanes_before_update(iVeh, :, :) ~= zeros(1, 2)

                for i = 1:length(obj.iter.lanes_before_update(iVeh, :, :))

                    if ~ismember(obj.scenario.vehicles(iVeh).lanelets_index(1), predicted_lanelets)
                        % prevent that first lane is no longer considered when adding lanes before update later
                        predicted_lanelets = [obj.scenario.vehicles(iVeh).lanelets_index(1), predicted_lanelets];
                    end

                    if ~ismember(obj.iter.lanes_before_update{iVeh}(1, i), predicted_lanelets)
                        predicted_lanelets = [obj.iter.lanes_before_update(iVeh, 1, i), predicted_lanelets];
                    end

                end

            end

            obj.iter.predicted_lanelets{iVeh} = predicted_lanelets;

            if visualize_trajectory_index_lab
                % visualize trajectory index
                visualization_point = 0;

                for i = 1:length(obj.iter.reference_trajectory_points(iVeh, :, :))
                    point.x = obj.iter.reference_trajectory_points(iVeh, i, 1);
                    point.y = obj.iter.reference_trajectory_points(iVeh, i, 2);
                    visualization_point = point;

                    color = Color;
                    color.r = uint8(240);
                    color.g = uint8(230);
                    color.b = uint8(26);

                    [visualization_command] = lab_visualizer(visualization_point, 'point', color);
                    exp.visualize(visualization_command);
                end

            end

            % Calculate the predicted lanelet boundary of vehicle iVeh based on its predicted lanelets
            predicted_lanelet_boundary = get_lanelets_boundary(predicted_lanelets, obj.scenario.lanelet_boundary, obj.scenario.vehicles(iVeh).lanelets_index, obj.scenario.options.environment, obj.scenario.vehicles(iVeh).is_loop);
            obj.iter.predicted_lanelet_boundary(iVeh, :) = predicted_lanelet_boundary;

            if visualize_boundaries_lab
                % visualize boundaries
                for i = 1:length(predicted_lanelet_boundary{1, 1})
                    left_boundary = predicted_lanelet_boundary{1, 1};
                    leftPoint.x = left_boundary(1, i);
                    leftPoint.y = left_boundary(2, i);
                    visualization_left_point = leftPoint;
                    color = Color;
                    color.r = uint8(170);
                    color.g = uint8(24);
                    color.b = uint8(186);
                    [visualization_command] = lab_visualizer(visualization_left_point, 'point', color);
                    exp.visualize(visualization_command);
                end

                for i = 1:length(predicted_lanelet_boundary{1, 2})
                    right_boundary = predicted_lanelet_boundary{1, 2};
                    rightPoint.x = right_boundary(1, i);
                    rightPoint.y = right_boundary(2, i);
                    visualization_right_point = rightPoint;
                    color = Color;
                    color.r = uint8(232);
                    color.g = uint8(111);
                    color.b = uint8(30);
                    [visualization_command] = lab_visualizer(visualization_right_point, 'point', color);
                    exp.visualize(visualization_command);
                end

            end

        end

        % Compute reachable sets for vehicle iVeh
        local_reachable_sets = obj.scenario.mpa.local_reachable_sets_conv;

        obj.iter.reachable_sets(iVeh, :) = get_reachable_sets(x0, y0, yaw0, local_reachable_sets(trim_current, :), predicted_lanelet_boundary, obj.scenario.options);

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

        mpa = obj.scenario.mpa;

        %% emergency maneuver for vehicle iVeh
        % emergency left maneuver (without offset)
        turn_left_area_without_offset = mpa.emergency_maneuvers{trim_current}.left{1};
        [turn_left_area_without_offset_x, turn_left_area_without_offset_y] = translate_global(yaw0, x0, y0, turn_left_area_without_offset(1, :), turn_left_area_without_offset(2, :));
        obj.iter.emergency_maneuvers{iVeh}.left_area_without_offset = [turn_left_area_without_offset_x; turn_left_area_without_offset_y];
        % emergency right maneuver (without offset)
        turn_right_area_without_offset = mpa.emergency_maneuvers{trim_current}.right{1};
        [turn_right_area_without_offset_x, turn_right_area_without_offset_y] = translate_global(yaw0, x0, y0, turn_right_area_without_offset(1, :), turn_right_area_without_offset(2, :));
        obj.iter.emergency_maneuvers{iVeh}.right_area_without_offset = [turn_right_area_without_offset_x; turn_right_area_without_offset_y];
        % emergency braking maneuver (without offset)
        braking_area_without_offset = mpa.emergency_maneuvers{trim_current}.braking_without_offset;
        [turn_braking_area_without_offset_x, turn_braking_area_without_offset_y] = translate_global(yaw0, x0, y0, braking_area_without_offset(1, :), braking_area_without_offset(2, :));
        obj.iter.emergency_maneuvers{iVeh}.braking_area_without_offset = [turn_braking_area_without_offset_x; turn_braking_area_without_offset_y];
        % emergency braking maneuver (with normal offset)
        braking_area = mpa.emergency_maneuvers{trim_current}.braking_with_offset;
        [turn_braking_area_x, turn_braking_area_y] = translate_global(yaw0, x0, y0, braking_area(1, :), braking_area(2, :));
        obj.iter.emergency_maneuvers{iVeh}.braking_area = [turn_braking_area_x; turn_braking_area_y];

        if obj.scenario.options.is_prioritized && obj.plant.amount == 1
            %% Send data to sync obj.iter for all vehicles (especially needed for priority assignment)
            obj.scenario.vehicles(iVeh).communicate.traffic.send_message(obj.k, obj.iter.x0(iVeh, :), obj.iter.trim_indices(iVeh), obj.iter.predicted_lanelets{iVeh}, obj.iter.occupied_areas{iVeh}, obj.iter.reachable_sets(iVeh, :));
        end

    end

    if obj.scenario.options.is_prioritized && obj.plant.amount == 1
        %% read messages from other vehicles (There shouldn't be any other vehicles if centralized)
        other_vehicles = setdiff(1:obj.scenario.options.amount, obj.plant.indices_in_vehicle_list);
        latest_msgs = read_messages(obj.scenario.vehicles(obj.plant.indices_in_vehicle_list(1)).communicate.traffic, obj.k, obj.scenario.options.amount - 1);

        for iVeh = other_vehicles
            %latest_msg_i = read_message(obj.scenario.vehicles(obj.plant.indices_in_vehicle_list(1)).communicate.traffic, obj.ros_subscribers.traffic{iVeh}, obj.k);
            latest_msg_i = latest_msgs(find([latest_msgs.vehicle_id] == obj.plant.all_veh_ids(iVeh), 1));
            obj.iter.x0(iVeh, :) = [latest_msg_i.current_pose.x, latest_msg_i.current_pose.y, latest_msg_i.current_pose.heading, latest_msg_i.current_pose.speed];
            obj.iter.trim_indices(iVeh) = latest_msg_i.current_trim_index;
            obj.iter.predicted_lanelets{iVeh} = latest_msg_i.predicted_lanelets';
            occupied_areas = latest_msg_i.occupied_areas;
            obj.iter.occupied_areas{iVeh}.normal_offset(1, :) = occupied_areas(1).x;
            obj.iter.occupied_areas{iVeh}.normal_offset(2, :) = occupied_areas(1).y;
            obj.iter.occupied_areas{iVeh}.without_offset(1, :) = occupied_areas(2).x;
            obj.iter.occupied_areas{iVeh}.without_offset(2, :) = occupied_areas(2).y;
            obj.iter.reachable_sets(iVeh, :) = (arrayfun(@(array) {polyshape(array.x, array.y)}, latest_msg_i.reachable_sets))';

            % Calculate the predicted lanelet boundary of vehicle iVeh based on its predicted lanelets
            % TODO is lanelets_index of other vehicles up to date?
            if (obj.scenario.options.scenario_type == ScenarioType.commonroad)
                predicted_lanelet_boundary = get_lanelets_boundary(obj.iter.predicted_lanelets{iVeh}, obj.scenario.lanelet_boundary, obj.scenario.vehicles(iVeh).lanelets_index, obj.scenario.options.environment, obj.scenario.vehicles(iVeh).is_loop);
                obj.iter.predicted_lanelet_boundary(iVeh, :) = predicted_lanelet_boundary;
            end

            x0 = obj.iter.x0(iVeh, idx.x);
            y0 = obj.iter.x0(iVeh, idx.y);
            yaw0 = obj.iter.x0(iVeh, idx.heading);
            trim_current = obj.iter.trim_indices(iVeh);

            %% emergency maneuver for vehicle iVeh
            % emergency left maneuver (without offset)
            turn_left_area_without_offset = mpa.emergency_maneuvers{trim_current}.left{1};
            [turn_left_area_without_offset_x, turn_left_area_without_offset_y] = translate_global(yaw0, x0, y0, turn_left_area_without_offset(1, :), turn_left_area_without_offset(2, :));
            obj.iter.emergency_maneuvers{iVeh}.left_area_without_offset = [turn_left_area_without_offset_x; turn_left_area_without_offset_y];
            % emergency right maneuver (without offset)
            turn_right_area_without_offset = mpa.emergency_maneuvers{trim_current}.right{1};
            [turn_right_area_without_offset_x, turn_right_area_without_offset_y] = translate_global(yaw0, x0, y0, turn_right_area_without_offset(1, :), turn_right_area_without_offset(2, :));
            obj.iter.emergency_maneuvers{iVeh}.right_area_without_offset = [turn_right_area_without_offset_x; turn_right_area_without_offset_y];
            % emergency braking maneuver (without offset)
            braking_area_without_offset = mpa.emergency_maneuvers{trim_current}.braking_without_offset;
            [turn_braking_area_without_offset_x, turn_braking_area_without_offset_y] = translate_global(yaw0, x0, y0, braking_area_without_offset(1, :), braking_area_without_offset(2, :));
            obj.iter.emergency_maneuvers{iVeh}.braking_area_without_offset = [turn_braking_area_without_offset_x; turn_braking_area_without_offset_y];
            % emergency braking maneuver (with normal offset)
            braking_area = mpa.emergency_maneuvers{trim_current}.braking_with_offset;
            [turn_braking_area_x, turn_braking_area_y] = translate_global(yaw0, x0, y0, braking_area(1, :), braking_area(2, :));
            obj.iter.emergency_maneuvers{iVeh}.braking_area = [turn_braking_area_x; turn_braking_area_y];
        end

    end

end
