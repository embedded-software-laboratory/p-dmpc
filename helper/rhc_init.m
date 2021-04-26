function iter = rhc_init(scenario, x_measured, trim_indices)
% Preprocessing step for RHC controller

    iter = struct;
    idx = indices();

    iter.referenceTrajectoryPoints = zeros(scenario.Hp,2,scenario.nVeh);
    iter.x0 = x_measured;
    
    iter.vRef = get_max_speeds(scenario.combined_graph);
    for iVeh=1:scenario.nVeh
        % Find equidistant points on the reference trajectory.
        iter.referenceTrajectoryPoints(:,:,iVeh) = sampleReferenceTrajectory(...
            scenario.Hp, ... % number of prediction steps
            scenario.vehicles(iVeh).referenceTrajectory, ...
            iter.x0(iVeh,idx.x), ... % vehicle position x
            iter.x0(iVeh,idx.y), ... % vehicle position y
            iter.vRef(iVeh)*scenario.dt...  % distance traveled in one timestep
        );
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