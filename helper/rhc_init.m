function iter = rhc_init(scenario, x_measured, trims_measured, options)
% RHC_INIT  Preprocessing step for RHC controller

    
    idx = indices();

    nVeh = scenario.nVeh;
    Hp = scenario.Hp;

    % initialize
    iter = struct;
    iter.referenceTrajectoryPoints = zeros(nVeh, Hp, 2);
    iter.referenceTrajectoryIndex = zeros(nVeh, Hp, 1);
    iter.x0 = zeros(nVeh, 4);                           % state
    iter.trim_indices = zeros(nVeh, 1);                 % current trim
    iter.vRef = zeros(nVeh,Hp);                         % reference speed
    iter.predicted_lanelets = cell(nVeh, 1);
    iter.predicted_lanelet_boundary = cell(nVeh, 3);    % first column for left boundary, second column for right boundary, third column for MATLAB polyshape instance
    iter.reachable_sets = cell(nVeh, Hp);               % cells to store instances of MATLAB calss `polyshape`

    % states of other vehicles can be directed measured
    iter.x0 = x_measured;

    for iVeh = 1:nVeh
        if options.isParl && strcmp(scenario.name, 'Commonroad')
            % In parallel computation, obtain the predicted trims and predicted
            % lanelets of other vehicles from the received messages
            timeout = 0.5;      is_timeout = true;
            read_start = tic;   read_time = toc(read_start);
            while read_time < timeout
                if scenario.ros_subscribers{iVeh}.LatestMessage.time_step == (scenario.k-1)
%                     disp(['Get current message after ' num2str(read_time) ' seconds.'])
                    is_timeout = false;
                    break
                end
                read_time = toc(read_start);
            end

            if is_timeout
                warning(['The predicted trims of vehicle ' num2str(iVeh) ' are unable to received.'...
                    'The pevious message will be used.'])
            end

            latest_message_i = scenario.ros_subscribers{iVeh}.LatestMessage;
            oldness_msg = scenario.k - latest_message_i.time_step;
            iter.trim_indices(iVeh) = latest_message_i.predicted_trims(oldness_msg+1);
        else
            % if parallel computation is not used, other vehicles' trims are measured 
            iter.trim_indices = trims_measured;
        end

        x0 = iter.x0(iVeh, idx.x);
        y0 = iter.x0(iVeh, idx.y);
        yaw0 = iter.x0(iVeh, idx.heading);
        trim_current = iter.trim_indices(iVeh);

        % Get the predicted lanelets of other vehicles
        [predicted_lanelets,reference,v_ref] = get_predicted_lanelets(scenario.vehicles(iVeh), trim_current, x0, y0, scenario.mpa, scenario.dt);

        % reference speed and path points
        iter.vRef(iVeh,:) = v_ref;
        
        % equidistant points on the reference trajectory.
        iter.referenceTrajectoryPoints(iVeh,:,:) = reference.ReferencePoints;
        iter.referenceTrajectoryIndex(iVeh,:,:) = reference.ReferenceIndex;
        
        if strcmp(scenario.name, 'Commonroad')
            if options.isParl
                % from received messages if parallel computation is used 
                latest_message_i = scenario.ros_subscribers{iVeh}.LatestMessage;
                predicted_lanelets= latest_message_i.predicted_lanelets(:)'; % make row vector
            end
            iter.predicted_lanelets{iVeh} = predicted_lanelets;
        
            % Calculate the predicted lanelet boundary of other vehicles based on their predicted lanelets
            predicted_lanelet_boundary = get_lanelets_boundary(predicted_lanelets, scenario.lanelet_boundary, scenario.vehicles(iVeh).lanelets_index);
            iter.predicted_lanelet_boundary(iVeh,:) = predicted_lanelet_boundary;
    
            if options.isParl
                % Calculate reachable sets of other vehicles based on their
                % current states and trims. Reachability analysis will be
                % widely used in the parallel computation.
                local_reachable_sets = scenario.mpa.local_reachable_sets;
                iter.reachable_sets(iVeh,:) = get_reachable_sets(x0, y0, yaw0, local_reachable_sets(trim_current,:), predicted_lanelet_boundary, scenario.is_allow_non_convex);
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