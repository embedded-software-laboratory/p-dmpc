classdef MotionGraph
    %UNTITLED3 Summary of this class goes here
    %   Detailed explanation goes here
    
    % trim
    %   u
    % maneuver
    %   dx
    %   dy
    %   dyaw
    %   area
    
    properties
        trims               % struct array
        maneuvers           % Cell/struct matrix (nTrims x nTrims)
        transition_matrix   % Matrix      (nTrims x nTrims)
    end
    
    methods
        function obj = MotionGraph(model, trim_inputs, trim_adjacency, dt)
            %UNTITLED3 Construct an instance of this class
            %   Detailed explanation goes here
            obj.transition_matrix = trim_adjacency;
            
            % maneuver state changes in local coordinates
            x0 = zeros(model.nx, 1);
            x0(4) = 3;
            [t, x] = ode45(@(t, x) model.ode(x, trim_inputs(:,1)), ...
                           [0 dt], ...
                           x0, ...
                           odeset('RelTol',1e-8,'AbsTol',1e-8));
        end
        
        function outputArg = method1(obj,inputArg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            outputArg = obj.Property1 + inputArg;
            
        end
    end
end

