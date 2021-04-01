function iter = rhc_init(scenario, x0)
% Preprocessing step for RHC controller

    iter = struct;
    idx = indices();

    iter.referenceTrajectoryPoints = zeros(scenario.Hp,2,scenario.nVeh);
    for v=1:scenario.nVeh
        % Find equidistant points on the reference trajectory.
        iter.referenceTrajectoryPoints(:,:,v) = sampleReferenceTrajectory(...
            scenario.Hp, ... % number of prediction steps
            scenario.vehicles(v).referenceTrajectory, ...
            x0(v,idx.x), ... % vehicle position x
            x0(v,idx.y), ... % vehicle position y
            x0(v,idx.speed)*scenario.dt);  % distance traveled in one timestep
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