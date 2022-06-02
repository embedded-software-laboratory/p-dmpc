function iter = rhc_init(scenario, x_measured, trims_measured, initialized_reference_path, is_sim_lab)
% RHC_INIT  Preprocessing step for RHC controller

    idx = indices();

    if ~is_sim_lab
        if ~initialized_reference_path
            for iVeh = 1:scenario.nVeh
                index = match_pose_to_lane(scenario, x_measured(iVeh, idx.x), x_measured(iVeh, idx.y));
                disp(sprintf("veh ID: %d, index: %d", scenario.vehicle_ids(iVeh), index));

                if scenario.manual_vehicle_id == scenario.vehicle_ids(iVeh)
                    if scenario.options.firstManualVehicleMode == 1
                        [updated_ref_path, scenario] = generate_manual_path(scenario, scenario.vehicle_ids(iVeh), 50, index, false);
                    else
                        continue
                    end     
                elseif scenario.second_manual_vehicle_id == scenario.vehicle_ids(iVeh)
                    if scenario.options.secondManualVehicleMode == 1
                        [updated_ref_path, scenario] = generate_manual_path(scenario, scenario.vehicle_ids(iVeh), 50, index, false);
                    else
                        continue
                    end      
                else
                    % ref_path = generate_ref_path(vehid(iveh));% function to generate refpath based on CPM Lab road geometry
                    [updated_ref_path, scenario] = generate_random_path(scenario, scenario.vehicle_ids(iVeh), 50, index); % function to generate random path for autonomous vehicles based on CPM Lab road geometry
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
        % Find equidistant points on the reference trajectory.
        reference = sampleReferenceTrajectory(...
            Hp, ... % number of prediction steps
            scenario.vehicles(iVeh).referenceTrajectory, ...
            iter.x0(iVeh, idx.x), ... % vehicle position x
            iter.x0(iVeh, idx.y), ... % vehicle position y
            iter.vRef(iVeh,:)*scenario.dt...  % distance traveled in one timestep
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
            iter.predicted_lanelets{iVeh} = predicted_lanelets;
        

            % Calculate the predicted lanelet boundary of other vehicles based on their predicted lanelets
            predicted_lanelet_boundary = get_lanelets_boundary(predicted_lanelets, scenario.lanelet_boundary);
            iter.predicted_lanelet_boundary(iVeh,:) = predicted_lanelet_boundary;
    
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