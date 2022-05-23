classdef Vehicle
% VEHICLE   Class representing vehicles in a scenario
    
    properties
        trim_config         = 1;     % initial trim configuration
        x_start             = 0;     % [m]
        y_start             = 0;     % [m]
        yaw_start           = 0;     % [radians]
        x_goal              = 0;     % [m]
        y_goal              = 0;     % [m]
        yaw_goal            = 0;     % [radians]
        referenceTrajectory = [0 0; 1 0; 3 1]; % [x1 y1; x2 y2; ...]
        Length              = 0.22;  % Vehicle length (bumper to bumper)[m]
        Width               = 0.1;   % Vehicle width [m]
        Lf                  = 0.1;   % Distance between vehicle center and front axle center [m]
        Lr                  = 0.1;   % Distance between vehicle center and rear axle center [m]
        ID                  = -1;    % vehicle ID (should be positive integer)
        lanelets_index;
        points_index;
        lanelet_boundary;                   % left and right boundaries of predicted lanelets
        communicate;                        % instance of the class `Communication`, used for communication via ROS 2
    end
    
    methods
        function obj = Vehicle()

        end
        
        function plot(obj, color)
            vehicle_polygon = transformedRectangle(obj.x_start,obj.y_start, obj.yaw_start, obj.Length, obj.Width);
            patch(vehicle_polygon(1,:),vehicle_polygon(2,:),color);
        end
    end
end
