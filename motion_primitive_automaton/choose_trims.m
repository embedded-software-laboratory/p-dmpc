function [trim_inputs, trim_adjacency] = choose_trims(trim_set)
% CHOOSE_TRIMS  Choose trim set used.
% NOTE: If new trim need to be added, please use successive number!

    switch trim_set
        case 1
            %% 3 trims: three speeds (0, 0.75 and 1.5 m/s), one steering (0 deg)
            speeds = 0:0.75:1.5;
            ntrims = numel(speeds);
            trim_inputs = zeros(ntrims,2);
            trim_inputs(:,2) = speeds;
            trim_adjacency = eye(ntrims);
            trim_adjacency(1:end-1,2:end) = trim_adjacency(1:end-1,2:end) ...
                + eye(ntrims-1);
            trim_adjacency(2:end,1:end-1) = trim_adjacency(2:end,1:end-1) ...
                + eye(ntrims-1);
    
        case 2
            % In manual mode: low speed profile
            %% 12 trims: two speeds (0, 0.05 and 0.6 m/s), 12 steering
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
    
        case 3
            %% 4 trims: two speeds (0 and 0.75 m/s), three steering
            steering = (-1:1) * pi/9;
            ntrims = numel(steering)+1;
            trim_inputs = zeros(ntrims,2);
            trim_inputs(2:end,1) = steering;
            trim_inputs(2:end,2) = 0.75;%10 / 3.6;
            % All states connected
            trim_adjacency = ones(ntrims);
    
        case 4
            % In manual mode: medium speed profile
            %% 12 trims: two speeds (0, 0.05 and 0.8 m/s), 12 steering
            steering = (-2:0.5:2) * pi/18;
            ntrims = numel(steering)+3;
            trim_inputs = zeros(ntrims,2);
            trim_inputs(2,1) = -0.6;
            trim_inputs(2,2) = 0.05;
            trim_inputs(end,1) = 0.6;
            trim_inputs(end,2) = 0.05;
            trim_inputs(3:end-1,1) = steering;
            trim_inputs(3:end-1,2) = 0.8;
            trim_adjacency = ones(ntrims);
            % equilibrium is state 1
            % equilibrium is reachable from all states
            % and can reach all states, so all 1s is good
            % other states are connected by one hop        
            trim_adjacency(2:end,2:end) = trim_adjacency(2:end,2:end) ...
                - triu(ones(ntrims-1),2)...
                - tril(ones(ntrims-1),-2);
    
        case 5
            %% 6 trims: two speeds (0 and 0.75 m/s), five steering
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
    
        case 6
            % In manual mode: high speed profile
            %% 12 trims: two speeds (0, 0.5 and 1.2 m/s), 12 steering
            steering = (-2:0.5:2) * pi/18;
            ntrims = numel(steering)+3;
            trim_inputs = zeros(ntrims,2);
            trim_inputs(2,1) = -0.6;
            trim_inputs(2,2) = 0.05;
            trim_inputs(end,1) = 0.6;
            trim_inputs(end,2) = 0.05;
            trim_inputs(3:end-1,1) = steering;
            trim_inputs(3:end-1,2) = 1.2;
            trim_adjacency = ones(ntrims);
            % equilibrium is state 1
            % equilibrium is reachable from all states
            % and can reach all states, so all 1s is good
            % other states are connected by one hop        
            trim_adjacency(2:end,2:end) = trim_adjacency(2:end,2:end) ...
                - triu(ones(ntrims-1),2)...
                - tril(ones(ntrims-1),-2);
    
        case 7
            %% 12 trims: two speeds (0, 0.5 and 0.6 m/s), 12 steering
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
            
        case 8
            %% 21 trims: four speeds (0, 0.05, 0.6 and 1.0 m/s), 12 steering
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
%     visualize_trims(trim_inputs,trim_adjacency)
end

%% local function
function visualize_trims(trim_inputs,trim_adjacency)
    figure
    angle = rad2deg(trim_inputs(:,1));
    speed = trim_inputs(:,2);
    G = digraph(trim_adjacency,'omitSelfLoops');
    p = plot(G,'XData',angle,'YData',speed,'MarkerSize',10);

    % workaround to change font size of node names
    p.NodeLabel = {};
    node_names = 1:length(angle);
    for i=1:length(node_names)
        text(p.XData(i)+0.8, p.YData(i), num2str(node_names(i)), 'FontSize', 20);
    end

    xlim([-45,45])
    ylim([-0.1,1.0])
    xlabel('\fontsize{14}Steering Angle \delta [\circ]','Interpreter','tex');
    ylabel('\fontsize{14}Speed \nu [m/s]','Interpreter','tex');
    grid on
end
