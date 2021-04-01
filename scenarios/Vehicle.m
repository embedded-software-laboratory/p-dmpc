classdef Vehicle < handle
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        trim_config = 1; % initial trim configuration
        x = 0; % [m]
        y = 0; % [m]
        yaw = 0; % [radians]
        referenceTrajectory = [0 0; 1 0; 3 1]; 
        Length = .98; % Vehicle length (bumper to bumper)[m]
        Width = .88; % Vehicle width [m]
        Lf = .34; % Distance between vehicle center and front axle center [m]
        Lr = .34; % Distance between vehicle center and rear axle center [m]
    end
    
    methods
        function obj = Vehicle()
            %UNTITLED2 Construct an instance of this class
            %   Detailed explanation goes here
        end
        
        function plot(obj, varargin)
            if nargin==1
                fig = figure;
                axis equal;
                color = 'b';
            else
                fig = varargin{1};
                color = varargin{2};
            end
            figure(fig);
            vehicle_polygon = transformedRectangle(obj.x,obj.y, obj.yaw, obj.Length, obj.Width);
            fill(vehicle_polygon(1,:),vehicle_polygon(2,:),color);
        end
    end
end

