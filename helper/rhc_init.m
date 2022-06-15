function iter = rhc_init(scenario, x_measured, trims_measured, initialized_reference_path, is_sim_lab, exp)
% RHC_INIT  Preprocessing step for RHC controller

    idx = indices();

    if ~is_sim_lab
        if ~initialized_reference_path
            for iVeh = 1:scenario.nVeh
                index = match_pose_to_lane(scenario, x_measured(iVeh, idx.x), x_measured(iVeh, idx.y));
                disp(sprintf("veh ID: %d, index: %d", scenario.vehicle_ids(iVeh), index));

                if scenario.manual_vehicle_id == scenario.vehicle_ids(iVeh)
                    if scenario.options.firstManualVehicleMode == 1
                        % function to generate random path for manual vehicles based on CPM Lab road geometry
                        [updated_ref_path, scenario] = generate_manual_path(scenario, scenario.vehicle_ids(iVeh), 10, index, false);
                    else
                        continue
                    end     
                elseif scenario.second_manual_vehicle_id == scenario.vehicle_ids(iVeh)
                    if scenario.options.secondManualVehicleMode == 1
                        % function to generate random path for manual vehicles based on CPM Lab road geometry
                        [updated_ref_path, scenario] = generate_manual_path(scenario, scenario.vehicle_ids(iVeh), 10, index, false);
                    else
                        continue
                    end      
                else
                    % function to generate random path for autonomous vehicles based on CPM Lab road geometry
                    [updated_ref_path, scenario, scenario.vehicles(iVeh).lane_change_indices, scenario.vehicles(iVeh).lane_change_lanes] = generate_random_path(scenario, scenario.vehicle_ids(iVeh), 10, index);
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
            end
        else
            for iVeh = 1:scenario.nVeh
                autoUpdatedPath = false;
                for i = 1:length(scenario.vehicles(iVeh).predicted_lanelets)
                    % if last lane is reached, then lane will be automatically updated
                    if scenario.vehicles(iVeh).predicted_lanelets(i) == scenario.vehicles(iVeh).lanelets_index(end-1)
                        if scenario.manual_vehicle_id == scenario.vehicle_ids(iVeh) && ~scenario.updated_manual_vehicle_path
                            if scenario.options.firstManualVehicleMode == 1
                                % function to generate random path for manual vehicles based on CPM Lab road geometry
                                [updated_ref_path, scenario] = generate_manual_path(scenario, scenario.vehicle_ids(iVeh), 10, scenario.vehicles(iVeh).lanelets_index(end-1), false);
                                autoUpdatedPath = true;
                            else
                                continue
                            end     
                        elseif scenario.second_manual_vehicle_id == scenario.vehicle_ids(iVeh) && ~scenario.updated_second_manual_vehicle_path
                            if scenario.options.secondManualVehicleMode == 1
                                % function to generate random path for manual vehicles based on CPM Lab road geometry
                                [updated_ref_path, scenario] = generate_manual_path(scenario, scenario.vehicle_ids(iVeh), 10, scenario.vehicles(iVeh).lanelets_index(end-1), false);
                                autoUpdatedPath = true;
                            else
                                continue
                            end      
                        else
                            % function to generate random path for autonomous vehicles based on CPM Lab road geometry
                            [updated_ref_path, scenario, scenario.vehicles(iVeh).lane_change_indices, scenario.vehicles(iVeh).lane_change_lanes] = generate_random_path(scenario, scenario.vehicle_ids(iVeh), 10, scenario.vehicles(iVeh).lanelets_index(end-1));
                            autoUpdatedPath = true;
                        end

                        % save lanes before update to add for boundaries
                        if autoUpdatedPath && length(scenario.vehicles(iVeh).lanelets_index) > 3
                            scenario.vehicles(iVeh).lanes_before_update(1,1) = scenario.vehicles(iVeh).lanelets_index(end-2);
                            scenario.vehicles(iVeh).lanes_before_update(1,2) = scenario.vehicles(iVeh).lanelets_index(end-3);
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
                    elseif scenario.vehicles(iVeh).predicted_lanelets(i) == scenario.vehicles(iVeh).lanelets_index(3)
                        % do not consider the lanes before path update for the boundaries anymore
                        scenario.vehicles(iVeh).lanes_before_update = zeros(1,2);
                    end
                end
            end
        end
    end

    nVeh = scenario.nVeh;
    Hp = scenario.Hp;
    
    iter = struct;
    iter.scenario = scenario;
    iter.referenceTrajectoryPoints = zeros(nVeh,Hp,2);
    iter.referenceTrajectoryIndex = zeros(nVeh,Hp,1);
    iter.x0 = zeros(nVeh, 4);                           % state
    iter.trim_indices = zeros(nVeh, 1);                 % current trim
    iter.vRef = zeros(nVeh,Hp);                         % reference speed  
    iter.predicted_lanelets = cell(nVeh, 1);
    iter.predicted_lanelet_boundary = cell(nVeh, 2);    % first column for left boundary, second column for right boundary
    iter.reachable_sets = cell(nVeh, Hp);               % cells to store instances of MATLAB calss `polyshape` 
    
    % states of other vehicles can be directed measured
    iter.x0 = x_measured;
    
    for iVeh=1:scenario.nVeh
        if scenario.options.isParl && strcmp(scenario.name, 'Commonroad')
            % In parallel computation, obtain the predicted trims and predicted
            % lanelets of other vehicles from the received messages
            latest_message_i = scenario.ros_subscribers{iVeh}.LatestMessage;
            msg_timestep = latest_message_i.time_step;
            
            % consider the oldness of the message
            iter.trim_indices(iVeh) = latest_message_i.predicted_trims(scenario.k-msg_timestep+1);
        else
            % if parallel computation is not used, other vehicles' trims are measured 
            iter.trim_indices = trims_measured;
        end

        iter.scenario.vehicles(iVeh).x_position = iter.x0(iVeh,idx.x);
        iter.scenario.vehicles(iVeh).y_position = iter.x0(iVeh,idx.y);
        if (scenario.manual_vehicle_id == scenario.vehicle_ids(iVeh) && scenario.manual_mpa_initialized)
            iter.vRef(iVeh,:) = get_max_speed(scenario.vehicles(iVeh).vehicle_mpa,iter.trim_indices(iVeh));
        elseif (scenario.second_manual_vehicle_id == scenario.vehicle_ids(iVeh) && scenario.second_manual_mpa_initialized)
            iter.vRef(iVeh,:) = get_max_speed(scenario.vehicles(iVeh).vehicle_mpa,iter.trim_indices(iVeh));
        else
            iter.vRef(iVeh,:) = get_max_speed(scenario.mpa,iter.trim_indices(iVeh));
        end

        if ~iter.scenario.options.is_sim_lab && iter.scenario.vehicle_ids(iVeh) ~= iter.scenario.manual_vehicle_id && iter.scenario.vehicle_ids(iVeh) ~= iter.scenario.second_manual_vehicle_id
            %[scenario.vehicles(iVeh).referenceTrajectory] = modify_lane_changes(scenario.vehicles(iVeh).lane_change_indices, iter.vRef(iVeh,:)*scenario.dt, scenario.vehicles(iVeh).referenceTrajectory);
        end

        % Find equidistant points on the reference trajectory.
        reference = sampleReferenceTrajectory(...
            Hp, ... % number of prediction steps
            scenario.vehicles(iVeh).referenceTrajectory, ...
            iter.x0(iVeh, idx.x), ... % vehicle position x
            iter.x0(iVeh, idx.y), ... % vehicle position y
            iter.vRef(iVeh,:)*scenario.dt, ...  % distance traveled in one timestep
            iVeh, ...
            exp, ...
            scenario...
        );
    
        iter.referenceTrajectoryPoints(iVeh,:,:) = reference.ReferencePoints;
        iter.referenceTrajectoryIndex(iVeh,:,:) = reference.ReferenceIndex;
        
        if strcmp(scenario.name, 'Commonroad')
            % Get the predicted lanelets of other vehicles
            if scenario.options.isParl
                % from received messages if parallel computation is used 
                latest_message_i = scenario.ros_subscribers{iVeh}.LatestMessage;
                predicted_lanelets= latest_message_i.predicted_lanelets(:)'; % make row vector
            else
                % otherwise, calculate the predicted lanelets for others
                ref_points_index = reshape(iter.referenceTrajectoryIndex(iVeh,:,:),Hp,1);
                predicted_lanelets = get_predicted_lanelets(scenario.vehicles(iVeh), ref_points_index, scenario.road_raw_data.lanelet);
            end

            % if random path was updated, include the last lane before updating, because the predicted lane are planned starting from the updated lane
            if scenario.vehicles(iVeh).lanes_before_update ~= zeros(1,2)
                for i = 1:length(scenario.vehicles(iVeh).lanes_before_update)
                    if ~ismember(scenario.vehicles(iVeh).lanes_before_update(1,i), predicted_lanelets)
                        predicted_lanelets = [scenario.vehicles(iVeh).lanes_before_update(1,i), predicted_lanelets];
                    end
                end
            end

            iter.predicted_lanelets{iVeh} = predicted_lanelets;
            iter.scenario.vehicles(iVeh).predicted_lanelets = iter.predicted_lanelets{iVeh};
            
            %{
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

                [visualization_command] = lab_visualize_point(scenario, visualization_point, iVeh, color);
                exp.visualize(visualization_command);
            end
            %}

            % Calculate the predicted lanelet boundary of other vehicles based on their predicted lanelets
            predicted_lanelet_boundary = get_lanelets_boundary(predicted_lanelets, scenario.lanelet_boundary);
            iter.predicted_lanelet_boundary(iVeh,:) = predicted_lanelet_boundary;

            %{
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
                [visualization_command] = lab_visualize_point(scenario, visualization_left_point, iVeh, color);
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
                [visualization_command] = lab_visualize_point(scenario, visualization_right_point, iVeh, color);
                exp.visualize(visualization_command);
            end
            %}
    
            if scenario.options.isParl
                % Calculate reachable sets of other vehicles based on their
                % current states and trims. Reachability analysis will be
                % widely used in the parallel computation.
                x0 = iter.x0(iVeh, idx.x);
                y0 = iter.x0(iVeh, idx.y);
                yaw0 = iter.x0(iVeh, idx.heading);
                trim_current = iter.trim_indices(iVeh);
                local_reachable_sets = scenario.mpa.local_reachable_sets;
                iter.reachable_sets(iVeh,:) = get_reachable_sets(x0, y0, yaw0, local_reachable_sets(trim_current,:), predicted_lanelet_boundary);
            end
        end
    end
   
    % Determine Obstacle positions (x = x0 + v*t)
    % iter.obstacleFutureTrajectories = zeros(scenario.nObst,2,Hp);
    % for k=1:Hp
    %     step = (k*scenario.dt+scenario.delay_x + scenario.dt + scenario.delay_u)*scenario.obstacles(:,idx.speed);
    %     iter.obstacleFutureTrajectories(:,idx.x,k) = step.*cos( scenario.obstacles(:,idx.heading) ) + obstacleState(:,idx.x);
    %     iter.obstacleFutureTrajectories(:,idx.y,k) = step.*sin( scenario.obstacles(:,idx.heading) ) + obstacleState(:,idx.y);
    % end
    
    % iter.uMax = uMax;

end