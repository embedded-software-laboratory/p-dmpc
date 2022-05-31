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
        x_position          = 0;
        y_position          = 0;
        referenceTrajectory = [0 0; 1 0; 3 1]; % [x1 y1; x2 y2; ...]
        Length              = 0.22;  % Vehicle length (bumper to bumper)[m]
        Width               = 0.1;   % Vehicle width [m]
        Lf                  = 0.1;   % Distance between vehicle center and front axle center [m]
        Lr                  = 0.1;   % Distance between vehicle center and rear axle center [m]
        lanelets_index
        predicted_lanelets
        points_index
        lanelet_boundary
        paddle_counter      = 2;     % initial speed profile
        vehicle_mpa;
        vehicle_id          = 0;     % initial vehicle id
    end
    
    methods
        function obj = Vehicle()
            %UNTITLED2 Construct an instance of this class
            %   Detailed explanation goes here
        end
        
        function plot(obj, color)
            vehicle_polygon = transformedRectangle(obj.x_start,obj.y_start, obj.yaw_start, obj.Length, obj.Width);
            patch(vehicle_polygon(1,:),vehicle_polygon(2,:),color);
        end
    end
end
