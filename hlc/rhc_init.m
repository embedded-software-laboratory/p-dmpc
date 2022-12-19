function [iter, iter_scenario] = rhc_init(scenario, x_measured, trims_measured, initialized_reference_path, ros_subscribers)
% RHC_INIT  Preprocessing step for RHC controller

    iter.k = k;

    idx = indices();
    visualize_trajectory_index_lab = false;
    visualize_boundaries_lab = false;
    predicted_lanelet_boundary = {[]};

    if scenario.options.is_mixed_traffic
        if ~initialized_reference_path
            for iVeh = 1:scenario.options.amount
                index = match_pose_to_lane(scenario, x_measured(iVeh, idx.x), x_measured(iVeh, idx.y));

                if scenario.manual_vehicle_id == scenario.vehicle_ids(iVeh)
                    if scenario.options.firstManualVehicleMode == 1
                        % function to generate random path for manual vehicles based on CPM Lab road geometry
                        updated_ref_path = generate_manual_path(scenario, scenario.vehicle_ids(iVeh), 10, index(1), false);
                    else
                        if scenario.options.isPB
                            % Communicate predicted trims, predicted lanelets and areas to other vehicles
                            predicted_trims = repmat(trims_measured(iVeh), 1, scenario.options.Hp+1); % current trim and predicted trims in the prediction horizon

                            % use index, as vehicle in Expert-Mode has no defined trajectory
                            predicted_lanelets = index;

                            predicted_occupied_areas = {}; % for initial time step, the occupied areas are not predicted yet
                            scenario.vehicles(iVeh).communicate.send_message(iter.k-1, predicted_trims, predicted_lanelets, predicted_occupied_areas); 
                        end

                        continue
                    end     
                elseif scenario.second_manual_vehicle_id == scenario.vehicle_ids(iVeh)
                    if scenario.options.secondManualVehicleMode == 1
                        % function to generate random path for manual vehicles based on CPM Lab road geometry
                        updated_ref_path = generate_manual_path(scenario, scenario.vehicle_ids(iVeh), 10, index(1), false);
                    else
                        if scenario.options.isPB
                            % Communicate predicted trims, predicted lanelets and areas to other vehicles
                            predicted_trims = repmat(trims_measured(iVeh), 1, scenario.options.Hp+1); % current trim and predicted trims in the prediction horizon

                            % use index, as vehicle in Expert-Mode has no defined trajectory
                            predicted_lanelets = index;

                            predicted_occupied_areas = {}; % for initial time step, the occupied areas are not predicted yet
                            scenario.vehicles(iVeh).communicate.send_message(iter.k-1, predicted_trims, predicted_lanelets, predicted_occupied_areas);  
                        end
                        
                        continue
                    end      
                else
                    if scenario.options.is_eval
                         % function to generate random path for autonomous vehicles based on CPM Lab road geometry
                        [updated_ref_path, iter.lane_change_indices(iVeh,:,:), iter.lane_change_lanes(iVeh,:,:)] = generate_random_path(scenario, scenario.vehicle_ids(iVeh), 10, 0);
                    else
                         % function to generate random path for autonomous vehicles based on CPM Lab road geometry
                        [updated_ref_path, iter.lane_change_indices(iVeh,:,:), iter.lane_change_lanes(iVeh,:,:)] = generate_random_path(scenario, scenario.vehicle_ids(iVeh), 10, index(1));
                    end
                end
                
                % TODO_DATA: Scenario changes here
                updatedRefPath = updated_ref_path.path;
                scenario.vehicles(iVeh).x_start = updatedRefPath(1,1);
                scenario.vehicles(iVeh).y_start = updatedRefPath(1,2);
                scenario.vehicles(iVeh).x_goal = updatedRefPath(2:end,1);
                scenario.vehicles(iVeh).y_goal = updatedRefPath(2:end,2);
                
                scenario.vehicles(iVeh).referenceTrajectory = [scenario.vehicles(iVeh).x_start scenario.vehicles(iVeh).y_start
                                        scenario.vehicles(iVeh).x_goal  scenario.vehicles(iVeh).y_goal];
                scenario.vehicles(iVeh).lanelets_index = updated_ref_path.lanelets_index;
                scenario.vehicles(iVeh).points_index = updated_ref_path.points_index;

                yaw = calculate_yaw(updatedRefPath);
                scenario.vehicles(iVeh).yaw_start = yaw(1);
                scenario.vehicles(iVeh).yaw_goal = yaw(2:end); 

                if scenario.options.isPB
                    % Communicate predicted trims, predicted lanelets and areas to other vehicles
                    predicted_trims = repmat(trims_measured(iVeh), 1, scenario.options.Hp+1); % current trim and predicted trims in the prediction horizon

                    x0 = x_measured(iVeh,idx.x); % vehicle position x
                    y0 = x_measured(iVeh,idx.y); % vehicle position y

                    predicted_lanelets = get_predicted_lanelets(scenario,iVeh,predicted_trims(1),x0,y0);

                    predicted_occupied_areas = {}; % for initial time step, the occupied areas are not predicted yet
                    scenario.vehicles(iVeh).communicate.send_message(iter.k-1, predicted_trims, predicted_lanelets, predicted_occupied_areas);   
                end        
            end
        else
            for iVeh = 1:scenario.options.amount
                iter.auto_updated_path(iVeh) = false;
                for i = 1:length(iter.predicted_lanelets{iVeh})
                    % if last lane is reached, then lane will be automatically updated
                    if iter.predicted_lanelets{iVeh}(i) == scenario.vehicles(iVeh).lanelets_index(end-1)
                        if scenario.manual_vehicle_id == scenario.vehicle_ids(iVeh) && ~scenario.updated_manual_vehicle_path
                            if scenario.options.firstManualVehicleMode == 1
                                % function to generate random path for manual vehicles based on CPM Lab road geometry
                                updated_ref_path = generate_manual_path(scenario, scenario.vehicle_ids(iVeh), 10, scenario.vehicles(iVeh).lanelets_index(end-1), false);
                                iter.auto_updated_path(iVeh) = true;
                            else
                                continue
                            end     
                        elseif scenario.second_manual_vehicle_id == scenario.vehicle_ids(iVeh) && ~scenario.updated_second_manual_vehicle_path
                            if scenario.options.secondManualVehicleMode == 1
                                % function to generate random path for manual vehicles based on CPM Lab road geometry
                                updated_ref_path = generate_manual_path(scenario, scenario.vehicle_ids(iVeh), 10, scenario.vehicles(iVeh).lanelets_index(end-1), false);
                                iter.auto_updated_path(iVeh) = true;
                            else
                                continue
                            end      
                        else
                            % function to generate random path for autonomous vehicles based on CPM Lab road geometry
                            [updated_ref_path, iter.lane_change_indices(iVeh,:,:), iter.lane_change_lanes(iVeh,:,:)] = generate_random_path(scenario, scenario.vehicle_ids(iVeh), 10, scenario.vehicles(iVeh).lanelets_index(end-1));
                            iter.auto_updated_path(iVeh) = true;
                        end

                        % save lanes before update to add for boundaries
                        if iter.auto_updated_path(iVeh) && length(scenario.vehicles(iVeh).lanelets_index) > 3
                            iter.lanes_before_update(iVeh,1,1) = scenario.vehicles(iVeh).lanelets_index(end-2);
                            iter.lanes_before_update(iVeh,1,2) = scenario.vehicles(iVeh).lanelets_index(end-3);
                        end
                        
                        updatedRefPath = updated_ref_path.path;
                        scenario.vehicles(iVeh).x_start = updatedRefPath(1,1);
                        scenario.vehicles(iVeh).y_start = updatedRefPath(1,2);
                        scenario.vehicles(iVeh).x_goal = updatedRefPath(2:end,1);
                        scenario.vehicles(iVeh).y_goal = updatedRefPath(2:end,2);
                        
                        scenario.vehicles(iVeh).referenceTrajectory = [scenario.vehicles(iVeh).x_start scenario.vehicles(iVeh).y_start
                                                scenario.vehicles(iVeh).x_goal  scenario.vehicles(iVeh).y_goal];
                        scenario.vehicles(iVeh).lanelets_index = updated_ref_path.lanelets_index;
                        scenario.vehicles(iVeh).points_index = updated_ref_path.points_index;

                        yaw = calculate_yaw(updatedRefPath);
                        scenario.vehicles(iVeh).yaw_start = yaw(1);
                        scenario.vehicles(iVeh).yaw_goal = yaw(2:end); 
                        break
                    elseif iter.predicted_lanelets{iVeh}(i) == scenario.vehicles(iVeh).lanelets_index(3)
                        % do not consider the lanes before path update for the boundaries anymore
                        iter.lanes_before_update(iVeh,:,:) = zeros(1,2);
                    end
                end
            end
        end
    end

    nVeh = scenario.options.amount;
    Hp = scenario.options.Hp;

    % states of other vehicles can be directed measured
    iter.x0 = x_measured;
    
    for iVeh=1:scenario.options.amount
        if scenario.options.isPB
            % In parallel computation, obtain the predicted trims and predicted
            % lanelets of other vehicles from the received messages
            latest_msg_i = read_message(scenario.vehicles(iVeh).communicate, ros_subscribers{iVeh}, scenario.k-1);
            oldness_msg = scenario.k - latest_msg_i.time_step;
            iter.trim_indices(iVeh) = latest_msg_i.predicted_trims(oldness_msg+1);
        else
            % if parallel computation is not used, other vehicles' trims are measured 
            iter.trim_indices = trims_measured;
        end

        x0 = iter.x0(iVeh, idx.x);
        y0 = iter.x0(iVeh, idx.y);
        yaw0 = iter.x0(iVeh, idx.heading);
        trim_current = iter.trim_indices(iVeh);

        % Get the predicted lanelets of other vehicles
        % Byproducts: reference path and reference speed profile
        [predicted_lanelets,reference,v_ref] = get_predicted_lanelets(scenario, iter, iVeh, trim_current, x0, y0);
%             iter.vRef(iVeh,:) = get_max_speed(scenario.mpa,iter.trim_indices(iVeh));

        if ~((scenario.options.veh_ids(iVeh) == str2double(scenario.options.mixed_traffic_config.first_manual_vehicle_id) && scenario.options.mixed_traffic_config.first_manual_vehicle_mode == Control_Mode.Expert_mode) ...
            || (scenario.options.veh_ids(iVeh) == str2double(scenario.options.mixed_traffic_config.second_manual_vehicle_id) && scenario.options.mixed_traffic_config.first_manual_vehicle_mode == Control_Mode.Expert_mode))
            % reference speed and path points
            iter.v_ref(iVeh,:) = v_ref;
            
            % equidistant points on the reference trajectory.
            iter.referenceTrajectoryPoints(iVeh,:,:) = reference.ReferencePoints;
            iter.referenceTrajectoryIndex(iVeh,:,:) = reference.ReferenceIndex;

           iter.last_trajectory_index(iVeh) = reference.ReferenceIndex(end);
        end
        
        if strcmp(scenario.options.scenario_name, 'Commonroad')

            % Vehicle in Expert-Mode does only consider boundaries when assuming RSS
            if ((scenario.options.veh_ids(iVeh) == str2double(scenario.options.mixed_traffic_config.first_manual_vehicle_id) && scenario.options.mixed_traffic_config.first_manual_vehicle_mode == Control_Mode.Expert_mode) ...
                || (scenario.options.veh_ids(iVeh) == str2double(scenario.options.mixed_traffic_config.second_manual_vehicle_id) && scenario.options.mixed_traffic_config.first_manual_vehicle_mode == Control_Mode.Expert_mode))

                iter.predicted_lanelets{iVeh} = predicted_lanelets;
                predicted_lanelet_boundary = cell(1, 3);

                if scenario.options.consider_RSS
                    % conisder boundary of predicted lanes
                    leftBound = [];
                    rightBound = [];
                    for i = 1:length(predicted_lanelets)
                        left_bound = struct2cell(scenario.road_raw_data.lanelet(predicted_lanelets(i)).leftBound.point);
                        right_bound = struct2cell(scenario.road_raw_data.lanelet(predicted_lanelets(i)).rightBound.point);
                        left_bound = cell2mat(left_bound);
                        right_bound = cell2mat(right_bound);

                        for j = 1:length(scenario.road_raw_data.lanelet(predicted_lanelets(i)).leftBound.point)
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
                % Get the predicted lanelets of other vehicles
                if scenario.options.isPB && ~scenario.vehicles(iVeh).autoUpdatedPath
                    % from received messages if parallel computation is used 
                    predicted_lanelets= latest_msg_i.predicted_lanelets(:)'; % make row vector
                end

                % if random path was updated, include the last lane before updating, because the predicted lane are planned starting from the updated lane
                if iter.lanes_before_update(iVeh,:,:) ~= zeros(1,2)
                    for i = 1:length(iter.lanes_before_update(iVeh,:,:))
                        if ~ismember(scenario.vehicles(iVeh).lanelets_index(1), predicted_lanelets)
                            % prevent that first lane is no longer considered when adding lanes before update later
                            predicted_lanelets = [scenario.vehicles(iVeh).lanelets_index(1), predicted_lanelets];
                        end

                        if ~ismember(iter.lanes_before_update{iVeh}(1,i), predicted_lanelets)
                            predicted_lanelets = [iter.lanes_before_update(iVeh,1,i), predicted_lanelets];
                        end
                    end
                end

                % if there is a lane change in the random path, add the boundary of the lane before the change as the vehicle might be still on the lane before change
                if ~scenario.options.is_sim_lab && scenario.manual_vehicle_id ~= scenario.vehicle_ids(iVeh) && scenario.second_manual_vehicle_id ~= scenario.vehicle_ids(iVeh)
                    if ~isempty(iter.lane_change_lanes(iVeh,:,:))
                       iter.lane_change_lanes(iVeh,:,:) = nonzeros(iter.lane_change_lanes(iVeh,:,:));
                        for i = 1:(length(iter.lane_change_lanes(iVeh,:,:)/2))
                            beforeLaneChange = scenario.vehicles(iVeh).lanelets_index(iter.lane_change_lanes(iVeh,i));
                            laneChange = scenario.vehicles(iVeh).lanelets_index(iter.lane_change_lanes(iVeh,i+(length(iter.lane_change_lanes(iVeh,:,:))/2)));
                            if ~ismember(beforeLaneChange, predicted_lanelets) && ismember(laneChange, predicted_lanelets)
                                predicted_lanelets = [beforeLaneChange, predicted_lanelets];
                            end
                        end
                    end
                end

                iter.predicted_lanelets{iVeh} = predicted_lanelets;
                
                if visualize_trajectory_index_lab
                    % visualize trajectory index
                    visualization_point = 0;
                    for i = 1:length(iter.referenceTrajectoryPoints(iVeh,:,:))
                        point.x = iter.referenceTrajectoryPoints(iVeh,i,1);
                        point.y = iter.referenceTrajectoryPoints(iVeh,i,2);
                        visualization_point = point;

                        color = Color;
                        color.r = uint8(240);
                        color.g = uint8(230);
                        color.b = uint8(26);

                        [visualization_command] = lab_visualizer(visualization_point, 'point', color);
                        exp.visualize(visualization_command);
                    end
                end

                % Calculate the predicted lanelet boundary of other vehicles based on their predicted lanelets
                predicted_lanelet_boundary = get_lanelets_boundary(predicted_lanelets, scenario.lanelet_boundary, scenario.vehicles(iVeh).lanelets_index, scenario.options.is_sim_lab, scenario.vehicles(iVeh).is_loop);
                iter.predicted_lanelet_boundary(iVeh,:) = predicted_lanelet_boundary;

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
    
            if scenario.options.amount > 1
                % Calculate reachable sets of other vehicles based on their
                % current states and trims. Reachability analysis will be
                % widely used in the parallel computation.
                if ((scenario.options.veh_ids(iVeh) == str2double(scenario.options.mixed_traffic_config.first_manual_vehicle_id)) && scenario.manual_mpa_initialized) ...
                    || ((scenario.options.veh_ids(iVeh) == str2double(scenario.options.mixed_traffic_config.second_manual_vehicle_id)) && scenario.second_manual_mpa_initialized)
                    local_reachable_sets = scenario.vehicles(iVeh).vehicle_mpa.local_reachable_sets;
                else
                    local_reachable_sets = scenario.mpa.local_reachable_sets_conv;
                end
                iter.reachable_sets(iVeh,:) = get_reachable_sets(x0, y0, yaw0, local_reachable_sets(trim_current,:), predicted_lanelet_boundary, scenario.options);
            end
        end

        if ((scenario.options.veh_ids(iVeh) == str2double(scenario.options.mixed_traffic_config.first_manual_vehicle_id)) && scenario.manual_mpa_initialized) ...
            || ((scenario.options.veh_ids(iVeh) == str2double(scenario.options.mixed_traffic_config.second_manual_vehicle_id)) && scenario.second_manual_mpa_initialized)
            local_reachable_sets = scenario.vehicles(iVeh).vehicle_mpa.local_reachable_sets;
        else
            local_reachable_sets = scenario.mpa.local_reachable_sets_conv;
        end
        iter.reachable_sets(iVeh,:) = get_reachable_sets(x0, y0, yaw0, local_reachable_sets(trim_current,:), predicted_lanelet_boundary, scenario.options);

        

        % get each vehicle's currently occupied area
        x_rec1 = [-1, -1,  1,  1, -1] * (scenario.vehicles(iVeh).Length/2 + scenario.options.offset); % repeat the first entry to enclose the shape
        y_rec1 = [-1,  1,  1, -1, -1] * (scenario.vehicles(iVeh).Width/2 + scenario.options.offset);
        % calculate displacement of model shape
        [x_rec2, y_rec2] = translate_global(yaw0, x0, y0, x_rec1, y_rec1);
        iter.occupied_areas{iVeh}.normal_offset = [x_rec2; y_rec2];

        x_rec1_without_offset = [-1, -1,  1,  1, -1] * (scenario.vehicles(iVeh).Length/2); % repeat the first entry to enclose the shape
        y_rec1_without_offset = [-1,  1,  1, -1, -1] * (scenario.vehicles(iVeh).Width/2);
        [x_rec2_without_offset, y_rec2_without_offset] = translate_global(yaw0, x0, y0, x_rec1_without_offset, y_rec1_without_offset);
        iter.occupied_areas{iVeh}.without_offset = [x_rec2_without_offset; y_rec2_without_offset];

        % Get vehicle's occupied area of emergency braking maneuver
        % with normal offset

        if ((scenario.options.veh_ids(iVeh) == str2double(scenario.options.mixed_traffic_config.first_manual_vehicle_id)) && scenario.manual_mpa_initialized) ...
            || ((scenario.options.veh_ids(iVeh) == str2double(scenario.options.mixed_traffic_config.second_manual_vehicle_id)) && scenario.second_manual_mpa_initialized)
            mpa = scenario.vehicles(iVeh).vehicle_mpa;
        else
            mpa = scenario.mpa;
        end

        % emergency left maneuver (without offset)
        turn_left_area_without_offset = mpa.emergency_maneuvers{trim_current}.left{1};
        [turn_left_area_without_offset_x,turn_left_area_without_offset_y] = translate_global(yaw0,x0,y0,turn_left_area_without_offset(1,:),turn_left_area_without_offset(2,:));
        iter.emergency_maneuvers{iVeh}.left_area_without_offset = [turn_left_area_without_offset_x;turn_left_area_without_offset_y];
        % emergency right maneuver (without offset)
        turn_right_area_without_offset = mpa.emergency_maneuvers{trim_current}.right{1};
        [turn_right_area_without_offset_x,turn_right_area_without_offset_y] = translate_global(yaw0,x0,y0,turn_right_area_without_offset(1,:),turn_right_area_without_offset(2,:));
        iter.emergency_maneuvers{iVeh}.right_area_without_offset = [turn_right_area_without_offset_x;turn_right_area_without_offset_y];
        % emergency braking maneuver (without offset)
        braking_area_without_offset = mpa.emergency_maneuvers{trim_current}.braking_without_offset;
        [turn_braking_area_without_offset_x,turn_braking_area_without_offset_y] = translate_global(yaw0,x0,y0,braking_area_without_offset(1,:),braking_area_without_offset(2,:));
        iter.emergency_maneuvers{iVeh}.braking_area_without_offset = [turn_braking_area_without_offset_x;turn_braking_area_without_offset_y];
        % emergency braking maneuver (with normal offset)
        braking_area = mpa.emergency_maneuvers{trim_current}.braking_with_offset;
        [turn_braking_area_x,turn_braking_area_y] = translate_global(yaw0,x0,y0,braking_area(1,:),braking_area(2,:));
        iter.emergency_maneuvers{iVeh}.braking_area = [turn_braking_area_x;turn_braking_area_y];
    end
   
    % Determine Obstacle positions (x = x0 + v*t)
    % iter.obstacleFutureTrajectories = zeros(scenario.nObst,2,Hp);
    % for k=1:Hp
    %     step = (k*scenario.options.dt+scenario.delay_x + scenario.options.dt + scenario.delay_u)*scenario.obstacles(:,idx.speed);
    %     iter.obstacleFutureTrajectories(:,idx.x,k) = step.*cos( scenario.obstacles(:,idx.heading) ) + obstacleState(:,idx.x);
    %     iter.obstacleFutureTrajectories(:,idx.y,k) = step.*sin( scenario.obstacles(:,idx.heading) ) + obstacleState(:,idx.y);
    % end
    
    % iter.uMax = uMax;

end