function rhc_init(obj, x_measured, trims_measured)
% RHC_INIT  Preprocessing step for RHC controller

    idx = indices();
    visualize_trajectory_index_lab = false;
    visualize_boundaries_lab = false;
    predicted_lanelet_boundary = {[]};

    if obj.scenario.options.is_mixed_traffic
        if ~obj.initialized_reference_path
            for iVeh = obj.indices_in_vehicle_list
                index = match_pose_to_lane(obj.scenario, x_measured(iVeh, idx.x), x_measured(iVeh, idx.y));

                if str2double(obj.scenario.options.mixed_traffic_config.first_manual_vehicle_id) == obj.scenario.options.veh_ids(iVeh)
                    if obj.scenario.options.mixed_traffic_config.first_manual_vehicle_mode == Control_Mode.Guided_mode
                        % function to generate random path for manual vehicles based on CPM Lab road geometry
                        updated_ref_path = generate_manual_path(obj.scenario, obj.scenario.options.veh_ids(iVeh), 10, index(1), false);
                    else
                        if obj.scenario.options.isPB
                            % Communicate predicted trims, predicted lanelets and areas to other vehicles
                            predicted_trims = repmat(trims_measured(iVeh), 1, obj.scenario.options.Hp+1); % current trim and predicted trims in the prediction horizon

                            % use index, as vehicle in Expert-Mode has no defined trajectory
                            predicted_lanelets = index;                            

                            predicted_occupied_areas = {}; % for initial time step, the occupied areas are not predicted yet
                            obj.scenario.vehicles(iVeh).communicate.predictions.send_message(obj.k-1, predicted_occupied_areas); 
                        end

                        continue
                    end     
                elseif str2double(obj.scenario.options.mixed_traffic_config.second_manual_vehicle_id) == obj.scenario.options.veh_ids(iVeh)
                    if obj.scenario.options.mixed_traffic_config.second_manual_vehicle_mode == Control_Mode.Guided_mode
                        % function to generate random path for manual vehicles based on CPM Lab road geometry
                        updated_ref_path = generate_manual_path(obj.scenario, obj.scenario.options.veh_ids(iVeh), 10, index(1), false);
                    else
                        if obj.scenario.options.isPB
                            % Communicate predicted trims, predicted lanelets and areas to other vehicles
                            predicted_trims = repmat(trims_measured(iVeh), 1, obj.scenario.options.Hp+1); % current trim and predicted trims in the prediction horizon

                            % use index, as vehicle in Expert-Mode has no defined trajectory
                            predicted_lanelets = index;

                            predicted_occupied_areas = {}; % for initial time step, the occupied areas are not predicted yet
                            obj.scenario.vehicles(iVeh).communicate.predictions.send_message(obj.k-1, predicted_occupied_areas);  
                        end
                        
                        continue
                    end      
                else
                    if obj.scenario.options.is_eval
                         % function to generate random path for autonomous vehicles based on CPM Lab road geometry
                        [updated_ref_path, obj.iter.lane_change_indices(iVeh,:,:), obj.iter.lane_change_lanes(iVeh,:,:)] = generate_random_path(obj.scenario, obj.scenario.options.veh_ids(iVeh), 10, 0);
                    else
                         % function to generate random path for autonomous vehicles based on CPM Lab road geometry
                         [updated_ref_path, obj.iter.lane_change_indices(iVeh,:,:), obj.iter.lane_change_lanes(iVeh,:,:)] = generate_random_path(obj.scenario, obj.scenario.options.veh_ids(iVeh), 10, index(1));
                    end
                end
                
                % TODO_DATA: Scenario changes here
                updatedRefPath = updated_ref_path.path;
                obj.scenario.vehicles(iVeh).x_start = updatedRefPath(1,1);
                obj.scenario.vehicles(iVeh).y_start = updatedRefPath(1,2);
                obj.scenario.vehicles(iVeh).x_goal = updatedRefPath(2:end,1);
                obj.scenario.vehicles(iVeh).y_goal = updatedRefPath(2:end,2);
                
                obj.scenario.vehicles(iVeh).referenceTrajectory = [obj.scenario.vehicles(iVeh).x_start obj.scenario.vehicles(iVeh).y_start
                                        obj.scenario.vehicles(iVeh).x_goal  obj.scenario.vehicles(iVeh).y_goal];
                obj.scenario.vehicles(iVeh).lanelets_index = updated_ref_path.lanelets_index;
                obj.scenario.vehicles(iVeh).points_index = updated_ref_path.points_index;

                yaw = calculate_yaw(updatedRefPath);
                obj.scenario.vehicles(iVeh).yaw_start = yaw(1);
                obj.scenario.vehicles(iVeh).yaw_goal = yaw(2:end); 

                if obj.scenario.options.isPB
                    % Communicate predicted trims, predicted lanelets and areas to other vehicles
                    predicted_trims = repmat(trims_measured(iVeh), 1, obj.scenario.options.Hp+1); % current trim and predicted trims in the prediction horizon

                    x0 = x_measured(iVeh,idx.x); % vehicle position x
                    y0 = x_measured(iVeh,idx.y); % vehicle position y

                    predicted_lanelets = get_predicted_lanelets(obj.scenario,iVeh,predicted_trims(1),x0,y0);

                    predicted_occupied_areas = {}; % for initial time step, the occupied areas are not predicted yet
                    obj.scenario.vehicles(iVeh).communicate.predictions.send_message(obj.k-1, predicted_trims, predicted_lanelets, predicted_occupied_areas);   
                end        
            end
        else
            for iVeh = obj.indices_in_vehicle_list
                obj.iter.vehicles(iVeh).autoUpdatedPath = false;
                for i = 1:length(obj.iter.predicted_lanelets{iVeh})
                    % if last lane is reached, then lane will be automatically updated
                    if obj.iter.predicted_lanelets{iVeh}(i) == obj.scenario.vehicles(iVeh).lanelets_index(end-1)
                        if str2double(obj.scenario.options.mixed_traffic_config.first_manual_vehicle_id) == obj.scenario.options.veh_ids(iVeh) && ~obj.scenario.updated_manual_vehicle_path
                            if obj.scenario.options.mixed_traffic_config.first_manual_vehicle_mode == Control_Mode.Guided_mode
                                % function to generate random path for manual vehicles based on CPM Lab road geometry
                                updated_ref_path = generate_manual_path(obj.scenario, obj.scenario.options.veh_ids(iVeh), 10, obj.scenario.vehicles(iVeh).lanelets_index(end-1), false);
                                obj.iter.auto_updated_path(iVeh) = true;
                            else
                                continue
                            end     
                        elseif str2double(obj.scenario.options.mixed_traffic_config.second_manual_vehicle_id) == obj.scenario.options.veh_ids(iVeh) && ~obj.scenario.updated_second_manual_vehicle_path
                            if obj.scenario.options.mixed_traffic_config.second_manual_vehicle_mode == Control_Mode.Guided_mode
                                % function to generate random path for manual vehicles based on CPM Lab road geometry
                                [updated_ref_path, obj.scenario] = generate_manual_path(obj.scenario, obj.scenario.options.veh_ids(iVeh), 10, obj.scenario.vehicles(iVeh).lanelets_index(end-1), false);
                                obj.iter.auto_updated_path(iVeh) = true;
                            else
                                continue
                            end      
                        else
                            % function to generate random path for autonomous vehicles based on CPM Lab road geometry
                            [updated_ref_path, obj.iter.lane_change_indices(iVeh,:,:), obj.iter.lane_change_lanes(iVeh,:,:)] = generate_random_path(obj.scenario, obj.scenario.options.veh_ids(iVeh), 10, obj.scenario.vehicles(iVeh).lanelets_index(end-1));
                            obj.iter.auto_updated_path(iVeh) = true;
                        end

                        % save lanes before update to add for boundaries
                        if obj.iter.auto_updated_path(iVeh) && length(obj.scenario.vehicles(iVeh).lanelets_index) > 3
                            obj.iter.lanes_before_update(iVeh,1,1) = obj.scenario.vehicles(iVeh).lanelets_index(end-2);
                            obj.iter.lanes_before_update(iVeh,1,2) = obj.scenario.vehicles(iVeh).lanelets_index(end-3);
                        end

                        updatedRefPath = updated_ref_path.path;
                        obj.scenario.vehicles(iVeh).x_start = updatedRefPath(1,1);
                        obj.scenario.vehicles(iVeh).y_start = updatedRefPath(1,2);
                        obj.scenario.vehicles(iVeh).x_goal = updatedRefPath(2:end,1);
                        obj.scenario.vehicles(iVeh).y_goal = updatedRefPath(2:end,2);
                        
                        obj.scenario.vehicles(iVeh).referenceTrajectory = [obj.scenario.vehicles(iVeh).x_start obj.scenario.vehicles(iVeh).y_start
                                                obj.scenario.vehicles(iVeh).x_goal  obj.scenario.vehicles(iVeh).y_goal];
                        obj.scenario.vehicles(iVeh).lanelets_index = updated_ref_path.lanelets_index;
                        obj.scenario.vehicles(iVeh).points_index = updated_ref_path.points_index;

                        yaw = calculate_yaw(updatedRefPath);
                        obj.scenario.vehicles(iVeh).yaw_start = yaw(1);
                        obj.scenario.vehicles(iVeh).yaw_goal = yaw(2:end); 
                        break
                    elseif obj.iter.predicted_lanelets{iVeh}(i) == obj.scenario.vehicles(iVeh).lanelets_index(3)
                        % do not consider the lanes before path update for the boundaries anymore
                        obj.iter.lanes_before_update(iVeh,:,:) = zeros(1,2);
                    end
                end
            end
        end
    end

    for iVeh = obj.indices_in_vehicle_list
        % states of controlled vehicles can be measured directly
        obj.iter.x0(iVeh,:) = x_measured(iVeh,:);
        % get own trim
        obj.iter.trim_indices(iVeh) = trims_measured(iVeh);
        x0 = obj.iter.x0(iVeh, idx.x);
        y0 = obj.iter.x0(iVeh, idx.y);
        yaw0 = obj.iter.x0(iVeh, idx.heading);
        trim_current = obj.iter.trim_indices(iVeh);

        % Compute the predicted lanelets of iVeh vehicle
        % Byproducts: reference path and reference speed profile
        [predicted_lanelets,reference,v_ref] = get_predicted_lanelets(obj.scenario, obj.iter, iVeh, x0, y0);


        % For non manual vehicles: Update ref speed and ref trajectory
        if ~((obj.scenario.options.veh_ids(iVeh) == str2double(obj.scenario.options.mixed_traffic_config.first_manual_vehicle_id) && obj.scenario.options.mixed_traffic_config.first_manual_vehicle_mode == Control_Mode.Expert_mode) ...
            || (obj.scenario.options.veh_ids(iVeh) == str2double(obj.scenario.options.mixed_traffic_config.second_manual_vehicle_id) && obj.scenario.options.mixed_traffic_config.second_manual_vehicle_mode == Control_Mode.Expert_mode))
            % reference speed and path points
            obj.iter.v_ref(iVeh,:) = v_ref;
            
            % equidistant points on the reference trajectory.
            obj.iter.referenceTrajectoryPoints(iVeh,:,:) = reference.ReferencePoints;
            obj.iter.referenceTrajectoryIndex(iVeh,:,:) = reference.ReferenceIndex;

            obj.iter.last_trajectory_index(iVeh) = reference.ReferenceIndex(end);
        end
        

        if obj.scenario.options.scenario_name == Scenario_Type.Commonroad

            % Vehicle in Expert-Mode does only consider boundaries when assuming RSS
            % use computed predicted_lanes and compute predicted_lanelet_boundary
            if ((obj.scenario.options.veh_ids(iVeh) == str2double(obj.scenario.options.mixed_traffic_config.first_manual_vehicle_id) && obj.scenario.options.mixed_traffic_config.first_manual_vehicle_mode == Control_Mode.Expert_mode) ...
                || (obj.scenario.options.veh_ids(iVeh) == str2double(obj.scenario.options.mixed_traffic_config.second_manual_vehicle_id) && obj.scenario.options.mixed_traffic_config.second_manual_vehicle_mode == Control_Mode.Expert_mode))

                obj.iter.predicted_lanelets{iVeh} = predicted_lanelets;
                predicted_lanelet_boundary = cell(1, 3);

                if obj.scenario.options.mixed_traffic_config.consider_rss
                    % conisder boundary of predicted lanes
                    leftBound = [];
                    rightBound = [];
                    for i = 1:length(predicted_lanelets)
                        left_bound = struct2cell(obj.scenario.road_raw_data.lanelet(predicted_lanelets(i)).leftBound.point);
                        right_bound = struct2cell(obj.scenario.road_raw_data.lanelet(predicted_lanelets(i)).rightBound.point);
                        left_bound = cell2mat(left_bound);
                        right_bound = cell2mat(right_bound);

                        for j = 1:length(obj.scenario.road_raw_data.lanelet(predicted_lanelets(i)).leftBound.point)
                            leftBound = [leftBound left_bound(:,:,j)];
                            rightBound = [rightBound right_bound(:,:,j)];
                        end
                    end

                    for i = 1:length(leftBound)
                        predicted_lanelet_boundary{1}(:,i) = leftBound(:,i);
                        predicted_lanelet_boundary{2}(:,i) = rightBound(:,i);
                    end
                end
            else
                % if random path was updated, include the last lane before updating, because the predicted lane are planned starting from the updated lane
                if obj.iter.lanes_before_update(iVeh,:,:) ~= zeros(1,2)
                    for i = 1:length(obj.iter.lanes_before_update(iVeh,:,:))
                        if ~ismember(obj.scenario.vehicles(iVeh).lanelets_index(1), predicted_lanelets)
                            % prevent that first lane is no longer considered when adding lanes before update later
                            predicted_lanelets = [obj.scenario.vehicles(iVeh).lanelets_index(1), predicted_lanelets];
                        end

                        if ~ismember(obj.iter.lanes_before_update{iVeh}(1,i), predicted_lanelets)
                            predicted_lanelets = [obj.iter.lanes_before_update(iVeh,1,i), predicted_lanelets];
                        end
                    end
                end

                % if there is a lane change in the random path, add the boundary of the lane before the change as the vehicle might be still on the lane before change
%                 if ~obj.scenario.options.is_sim_lab && str2double(obj.scenario.options.mixed_traffic_config.first_manual_vehicle_id) ~= obj.scenario.options.veh_ids(iVeh) && str2double(obj.scenario.options.mixed_traffic_config.second_manual_vehicle_id) ~= obj.scenario.options.veh_ids(iVeh)
%                     if ~isempty(obj.iter.lane_change_lanes(iVeh,:,:))
%                         obj.iter.lane_change_lanes(iVeh,:,:) = nonzeros(obj.iter.lane_change_lanes(iVeh,:,:));
%                         for i = 1:(length(obj.iter.lane_change_lanes(iVeh,:,:))/2)
%                             beforeLaneChange = obj.scenario.vehicles(iVeh).lanelets_index(obj.iter.lane_change_lanes(iVeh,i));
%                             laneChange = obj.scenario.vehicles(iVeh).lanelets_index(obj.iter.lane_change_lanes(iVeh,i+(length(obj.iter.lane_change_lanes(iVeh,:,:)))/2));
%                             if ~ismember(beforeLaneChange, predicted_lanelets) && ismember(laneChange, predicted_lanelets)
%                                 predicted_lanelets = [beforeLaneChange, predicted_lanelets];
%                             end
%                         end
%                     end
%                 end

                obj.iter.predicted_lanelets{iVeh} = predicted_lanelets;
                
                if visualize_trajectory_index_lab
                    % visualize trajectory index
                    visualization_point = 0;
                    for i = 1:length(obj.iter.referenceTrajectoryPoints(iVeh,:,:))
                        point.x = obj.iter.referenceTrajectoryPoints(iVeh,i,1);
                        point.y = obj.iter.referenceTrajectoryPoints(iVeh,i,2);
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
                predicted_lanelet_boundary = get_lanelets_boundary(predicted_lanelets, obj.scenario.lanelet_boundary, obj.scenario.vehicles(iVeh).lanelets_index, obj.scenario.options.is_sim_lab, obj.scenario.vehicles(iVeh).is_loop);
                obj.iter.predicted_lanelet_boundary(iVeh,:) = predicted_lanelet_boundary;

                if visualize_boundaries_lab
                    % visualize boundaries
                    for i = 1:length(predicted_lanelet_boundary{1,1})
                        left_boundary = predicted_lanelet_boundary{1,1};
                        leftPoint.x = left_boundary(1,i);
                        leftPoint.y = left_boundary(2,i);
                        visualization_left_point = leftPoint;
                        color = Color;
                        color.r = uint8(170);
                        color.g = uint8(24);
                        color.b = uint8(186);
                        [visualization_command] = lab_visualizer(visualization_left_point, 'point', color);
                        exp.visualize(visualization_command);
                    end

                    for i = 1:length(predicted_lanelet_boundary{1,2})
                        right_boundary = predicted_lanelet_boundary{1,2};
                        rightPoint.x = right_boundary(1,i);
                        rightPoint.y = right_boundary(2,i);
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
        end

        % Compute reachable sets for vehicle iVeh
        if ((obj.scenario.options.veh_ids(iVeh) == str2double(obj.scenario.options.mixed_traffic_config.first_manual_vehicle_id)) && obj.scenario.manual_mpa_initialized) ...
                || ((obj.scenario.options.veh_ids(iVeh) == str2double(obj.scenario.options.mixed_traffic_config.second_manual_vehicle_id)) && obj.scenario.second_manual_mpa_initialized)
            local_reachable_sets = obj.scenario.vehicles(iVeh).vehicle_mpa.local_reachable_sets;
        else
            local_reachable_sets = obj.scenario.mpa.local_reachable_sets_conv;
        end
        obj.iter.reachable_sets(iVeh,:) = get_reachable_sets(x0, y0, yaw0, local_reachable_sets(trim_current,:), predicted_lanelet_boundary, obj.scenario.options);

        % get vehicles currently occupied area
        x_rec1 = [-1, -1,  1,  1, -1] * (obj.scenario.vehicles(iVeh).Length/2 + obj.scenario.options.offset); % repeat the first entry to enclose the shape
        y_rec1 = [-1,  1,  1, -1, -1] * (obj.scenario.vehicles(iVeh).Width/2 + obj.scenario.options.offset);
        % calculate displacement of model shape
        [x_rec2, y_rec2] = translate_global(yaw0, x0, y0, x_rec1, y_rec1);
        obj.iter.occupied_areas{iVeh}.normal_offset = [x_rec2; y_rec2];

        x_rec1_without_offset = [-1, -1,  1,  1, -1] * (obj.scenario.vehicles(iVeh).Length/2); % repeat the first entry to enclose the shape
        y_rec1_without_offset = [-1,  1,  1, -1, -1] * (obj.scenario.vehicles(iVeh).Width/2);
        [x_rec2_without_offset, y_rec2_without_offset] = translate_global(yaw0, x0, y0, x_rec1_without_offset, y_rec1_without_offset);
        obj.iter.occupied_areas{iVeh}.without_offset = [x_rec2_without_offset; y_rec2_without_offset];

        % Get vehicle's occupied area of emergency braking maneuver
        % with normal offset

        if ((obj.scenario.options.veh_ids(iVeh) == str2double(obj.scenario.options.mixed_traffic_config.first_manual_vehicle_id)) && obj.scenario.manual_mpa_initialized) ...
            || ((obj.scenario.options.veh_ids(iVeh) == str2double(obj.scenario.options.mixed_traffic_config.second_manual_vehicle_id)) && obj.scenario.second_manual_mpa_initialized)
            mpa = obj.scenario.vehicles(iVeh).vehicle_mpa;
        else
            mpa = obj.scenario.mpa;
        end

        %% emergency maneuver for vehicle iVeh
        % emergency left maneuver (without offset)
        turn_left_area_without_offset = mpa.emergency_maneuvers{trim_current}.left{1};
        [turn_left_area_without_offset_x,turn_left_area_without_offset_y] = translate_global(yaw0,x0,y0,turn_left_area_without_offset(1,:),turn_left_area_without_offset(2,:));
        obj.iter.emergency_maneuvers{iVeh}.left_area_without_offset = [turn_left_area_without_offset_x;turn_left_area_without_offset_y];
        % emergency right maneuver (without offset)
        turn_right_area_without_offset = mpa.emergency_maneuvers{trim_current}.right{1};
        [turn_right_area_without_offset_x,turn_right_area_without_offset_y] = translate_global(yaw0,x0,y0,turn_right_area_without_offset(1,:),turn_right_area_without_offset(2,:));
        obj.iter.emergency_maneuvers{iVeh}.right_area_without_offset = [turn_right_area_without_offset_x;turn_right_area_without_offset_y];
        % emergency braking maneuver (without offset)
        braking_area_without_offset = mpa.emergency_maneuvers{trim_current}.braking_without_offset;
        [turn_braking_area_without_offset_x,turn_braking_area_without_offset_y] = translate_global(yaw0,x0,y0,braking_area_without_offset(1,:),braking_area_without_offset(2,:));
        obj.iter.emergency_maneuvers{iVeh}.braking_area_without_offset = [turn_braking_area_without_offset_x;turn_braking_area_without_offset_y];
        % emergency braking maneuver (with normal offset)
        braking_area = mpa.emergency_maneuvers{trim_current}.braking_with_offset;
        [turn_braking_area_x,turn_braking_area_y] = translate_global(yaw0,x0,y0,braking_area(1,:),braking_area(2,:));
        obj.iter.emergency_maneuvers{iVeh}.braking_area = [turn_braking_area_x;turn_braking_area_y];


        if obj.scenario.options.isPB && obj.amount == 1
            %% Send data to sync obj.iter for all vehicles (especially needed for priority assignment)
            obj.scenario.vehicles(iVeh).communicate.traffic.send_message(obj.k, obj.iter.x0(iVeh,:), obj.iter.trim_indices(iVeh), obj.iter.predicted_lanelets{iVeh}, obj.iter.occupied_areas{iVeh}, obj.iter.reachable_sets(iVeh,:));
        end

    end

    if obj.scenario.options.isPB && obj.amount == 1
        %% read messages from other vehicles (There shouldn't be any other vehicles if centralized)
        other_vehicles = setdiff(1:obj.scenario.options.amount, obj.indices_in_vehicle_list);
        latest_msgs = read_messages(obj.scenario.vehicles(obj.indices_in_vehicle_list(1)).communicate.traffic, obj.k, obj.scenario.options.amount - 1);
        for iVeh = other_vehicles
            %latest_msg_i = read_message(obj.scenario.vehicles(obj.indices_in_vehicle_list(1)).communicate.traffic, obj.ros_subscribers.traffic{iVeh}, obj.k);
            latest_msg_i = latest_msgs(find([latest_msgs.vehicle_id] == obj.scenario.options.veh_ids(iVeh),1));
            obj.iter.x0(iVeh,:) = [latest_msg_i.current_pose.x, latest_msg_i.current_pose.y, latest_msg_i.current_pose.heading, latest_msg_i.current_pose.speed];
            obj.iter.trim_indices(iVeh) = latest_msg_i.current_trim_index;
            obj.iter.predicted_lanelets{iVeh} = latest_msg_i.predicted_lanelets';
            occupied_areas = latest_msg_i.occupied_areas;
            obj.iter.occupied_areas{iVeh}.normal_offset(1,:) = occupied_areas(1).x;
            obj.iter.occupied_areas{iVeh}.normal_offset(2,:) = occupied_areas(1).y;
            obj.iter.occupied_areas{iVeh}.without_offset(1,:) = occupied_areas(2).x;
            obj.iter.occupied_areas{iVeh}.without_offset(2,:) = occupied_areas(2).y;
            obj.iter.reachable_sets(iVeh,:) = (arrayfun(@(array) {polyshape(array.x,array.y)}, latest_msg_i.reachable_sets))';

            % Calculate the predicted lanelet boundary of vehicle iVeh based on its predicted lanelets
            % TODO is lanelets_index of other vehicles up to date?
            if obj.scenario.options.scenario_name == Scenario_Type.Commonroad
                predicted_lanelet_boundary = get_lanelets_boundary(obj.iter.predicted_lanelets{iVeh}, obj.scenario.lanelet_boundary, obj.scenario.vehicles(iVeh).lanelets_index, obj.scenario.options.is_sim_lab, obj.scenario.vehicles(iVeh).is_loop);
                obj.iter.predicted_lanelet_boundary(iVeh,:) = predicted_lanelet_boundary;
            end

            x0 = obj.iter.x0(iVeh, idx.x);
            y0 = obj.iter.x0(iVeh, idx.y);
            yaw0 = obj.iter.x0(iVeh, idx.heading);
            trim_current = obj.iter.trim_indices(iVeh);

            %% emergency maneuver for vehicle iVeh
            % emergency left maneuver (without offset)
            turn_left_area_without_offset = mpa.emergency_maneuvers{trim_current}.left{1};
            [turn_left_area_without_offset_x,turn_left_area_without_offset_y] = translate_global(yaw0,x0,y0,turn_left_area_without_offset(1,:),turn_left_area_without_offset(2,:));
            obj.iter.emergency_maneuvers{iVeh}.left_area_without_offset = [turn_left_area_without_offset_x;turn_left_area_without_offset_y];
            % emergency right maneuver (without offset)
            turn_right_area_without_offset = mpa.emergency_maneuvers{trim_current}.right{1};
            [turn_right_area_without_offset_x,turn_right_area_without_offset_y] = translate_global(yaw0,x0,y0,turn_right_area_without_offset(1,:),turn_right_area_without_offset(2,:));
            obj.iter.emergency_maneuvers{iVeh}.right_area_without_offset = [turn_right_area_without_offset_x;turn_right_area_without_offset_y];
            % emergency braking maneuver (without offset)
            braking_area_without_offset = mpa.emergency_maneuvers{trim_current}.braking_without_offset;
            [turn_braking_area_without_offset_x,turn_braking_area_without_offset_y] = translate_global(yaw0,x0,y0,braking_area_without_offset(1,:),braking_area_without_offset(2,:));
            obj.iter.emergency_maneuvers{iVeh}.braking_area_without_offset = [turn_braking_area_without_offset_x;turn_braking_area_without_offset_y];
            % emergency braking maneuver (with normal offset)
            braking_area = mpa.emergency_maneuvers{trim_current}.braking_with_offset;
            [turn_braking_area_x,turn_braking_area_y] = translate_global(yaw0,x0,y0,braking_area(1,:),braking_area(2,:));
            obj.iter.emergency_maneuvers{iVeh}.braking_area = [turn_braking_area_x;turn_braking_area_y];
        end
    end

    if ( obj.scenario.options.amount > 1 )
        % update the coupling information
        obj.iter = coupling_based_on_reachable_sets(obj.scenario, obj.iter); % TODO_DATA: Scenario changes here
    end
end