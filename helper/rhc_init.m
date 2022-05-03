function iter = rhc_init(scenario, x_measured, trim_indices, initialized_reference_path, mVehid, isPB, vehid, is_sim_lab)
% RHC_INIT  Preprocessing step for RHC controller

    idx = indices();

    if ~is_sim_lab
        if ~initialized_reference_path
            for iVeh = 1:scenario.nVeh
                index = match_pose_to_lane(x_measured(iVeh, idx.x), x_measured(iVeh, idx.y));
                disp(sprintf("veh ID: %d, index: %d", vehid(iVeh), index));

                if (mVehid == vehid(iVeh))
                    updated_ref_path = generate_manual_path(scenario, vehid(iVeh), 20, index);     
                else
                    % ref_path = generate_ref_path(vehid(iveh));% function to generate refpath based on CPM Lab road geometry
                    updated_ref_path = generate_random_path(scenario, vehid(iVeh), 20, index); % function to generate random path for autonomous vehicles based on CPM Lab road geometry
                end
                
                updatedRefPath = updated_ref_path.path;
                %{
                if is_CPM_lab
                    updatedRefPathCopy = updatedRefPath(:,:);
                    disp(updatedRefPath(:,1));
                    disp(updatedRefPath(:,2));
                    %updatedRefPath(:,1) = updatedRefPathCopy(:,2);
                    %updatedRefPath(:,2) = updatedRefPathCopy(:,1);
                    %disp(updatedRefPath(:,1));
                    %disp(updatedRefPath(:,2));
            
                    sub = zeros(length(updatedRefPath),2);
                    for i = 1:length(updatedRefPath)
                        sub(i,1) = 4;
                    end
            
                    for i = 1:length(updatedRefPath)
                        updatedRefPath(i,1) = sub(i,1) - updatedRefPath(i,1);
                    end
                    disp(updatedRefPath(:,1));
                end
                %}

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
    
    iter = struct;
    iter.scenario = scenario;
    iter.referenceTrajectoryPoints = zeros(scenario.nVeh,scenario.Hp,2);
    iter.referenceTrajectoryIndex = zeros(scenario.nVeh,scenario.Hp,1);
    iter.x0 = x_measured;
    iter.trim_indices = trim_indices;
    
    iter.vRef = zeros(scenario.nVeh,scenario.Hp);
    for iVeh=1:scenario.nVeh
        iter.vRef(iVeh,:) = get_max_speed(scenario.mpa,trim_indices(iVeh));
        % Find equidistant points on the reference trajectory.
        reference = sampleReferenceTrajectory(...
            scenario.Hp, ... % number of prediction steps
            scenario.vehicles(iVeh).referenceTrajectory, ...
            iter.x0(iVeh,idx.x), ... % vehicle position x
            iter.x0(iVeh,idx.y), ... % vehicle position y
            iter.vRef(iVeh,:)*scenario.dt...  % distance traveled in one timestep
        );
    
        iter.referenceTrajectoryPoints(iVeh,:,:) = reference.ReferencePoints;
        iter.referenceTrajectoryIndex(iVeh,:,:) = reference.ReferenceIndex;
    
    end
    
   
    % Determine Obstacle positions (x = x0 + v*t)
    % iter.obstacleFutureTrajectories = zeros(scenario.nObst,2,scenario.Hp);
    % for k=1:scenario.Hp
    %     step = (k*scenario.dt+scenario.delay_x + scenario.dt + scenario.delay_u)*scenario.obstacles(:,idx.speed);
    %     iter.obstacleFutureTrajectories(:,idx.x,k) = step.*cos( scenario.obstacles(:,idx.heading) ) + obstacleState(:,idx.x);
    %     iter.obstacleFutureTrajectories(:,idx.y,k) = step.*sin( scenario.obstacles(:,idx.heading) ) + obstacleState(:,idx.y);
    % end
    
    % iter.uMax = uMax;

end