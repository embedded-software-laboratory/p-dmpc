function [trim_inputs, trim_adjacency] = choose_trims(trim_set)
% CHOOSE_TRIMS  Choose trim set used.

    switch trim_set
    case 1
        %% Three speeds (0 to 1.5 m/s), one steering (0 deg)
        speeds = 0:0.75:1.5;
        ntrims = numel(speeds);
        trim_inputs = zeros(ntrims,2);
        trim_inputs(:,2) = speeds;
        trim_adjacency = eye(ntrims);
        trim_adjacency(1:end-1,2:end) = trim_adjacency(1:end-1,2:end) ...
            + eye(ntrims-1);
        trim_adjacency(2:end,1:end-1) = trim_adjacency(2:end,1:end-1) ...
            + eye(ntrims-1);
    case 3
        %% Two speeds (0 and 0.75 m/s), three steering
        steering = (-1:1) * pi/9;
        ntrims = numel(steering)+1;
        trim_inputs = zeros(ntrims,2);
        trim_inputs(2:end,1) = steering;
        trim_inputs(2:end,2) = 0.75;%10 / 3.6;
        % All states connected
        trim_adjacency = ones(ntrims);
    case 5
        %% Two speeds (0 and 0.75 m/s), five steering
        steering = (-2:2) * pi/18;
        ntrims = numel(steering)+1;
        trim_inputs = zeros(ntrims,2);
        trim_inputs(2:end,1) = steering;
        trim_inputs(2:end,2) = 0.75;%10 / 3.6;
        trim_adjacency = ones(ntrims);
        % equilibrium is state 1
        % equilibrium is reachable from all states
        % and can reach all states, so all 1s is good
        % other states are connected by one hop
        trim_adjacency(2:end,2:end) = trim_adjacency(2:end,2:end) ...
            - triu(ones(ntrims-1),2)...
            - tril(ones(ntrims-1),-2);
    case 12
        %% Two speeds (0 and 0.6 m/s), 12 steering
        steering = (-2:0.5:2) * pi/18;
        ntrims = numel(steering)+3;
        trim_inputs = zeros(ntrims,2);
        trim_inputs(2,1) = -0.6;
        trim_inputs(2,2) = 0.05;
        trim_inputs(end,1) = 0.6;
        trim_inputs(end,2) = 0.05;
        trim_inputs(3:end-1,1) = steering;
        trim_inputs(3:end-1,2) = 0.6;
        trim_adjacency = ones(ntrims);
        % equilibrium is state 1
        % equilibrium is reachable from all states
        % and can reach all states, so all 1s is good
        % other states are connected by one hop        
        trim_adjacency(2:end,2:end) = trim_adjacency(2:end,2:end) ...
            - triu(ones(ntrims-1),2)...
            - tril(ones(ntrims-1),-2);
        
    case 13
        %% Two speeds (0, 0.6 m/s), 12 steering
        steering = [-2,-2,-1.5,-1.5,-1,-1,-0.5,-0.5,0,0,0.5,0.5,1,1,1.5,1.5,2,2] * pi/18;
        ntrims = numel(steering)+3;
        trim_inputs = zeros(ntrims,2);
        trim_inputs(2,1) = -0.6;
        trim_inputs(2,2) = 0.05;
        trim_inputs(end,1) = 0.6;
        trim_inputs(end,2) = 0.05;
        trim_inputs(3:end-1,1) = steering;
        trim_inputs(3:end-1,2) = [0.6,1.0,0.6,1.0,0.6,1.0,0.6,1.0,0.6,1.0,0.6,1.0,0.6,1.0,0.6,1.0,0.6,1.0];
        trim_adjacency = ones(ntrims);
        % equilibrium is state 1
        % equilibrium is reachable from all states
        % and can reach all states, so all 1s is good
        % other states are connected by one hop       
        trim_adjacency(2:end,2:end) = trim_adjacency(2:end,2:end) ...
            - triu(ones(ntrims-1),3)...
            - tril(ones(ntrims-1),-4);
    end
end