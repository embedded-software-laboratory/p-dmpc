% MIT License
% 
% Copyright (c) 2021 Lehrstuhl Informatik 11 - RWTH Aachen University
% 
% Permission is hereby granted, free of charge, to any person obtaining a copy
% of this software and associated documentation files (the "Software"), to deal
% in the Software without restriction, including without limitation the rights
% to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
% copies of the Software, and to permit persons to whom the Software is
% furnished to do so, subject to the following conditions:
% 
% The above copyright notice and this permission notice shall be included in all
% copies or substantial portions of the Software.
% 
% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
% IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
% FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
% AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
% LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
% OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
% SOFTWARE.
% 
% This file is part of receding-horizon-graph-search.
% 
% Author: i11 - Embedded Software, RWTH Aachen University

classdef Vehicle
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here
    
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

