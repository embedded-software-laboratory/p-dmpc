classdef PlantMeasurement
    % PlantMeasurement class that defines the required information for the
    % HighLevelController to receive at the measure step

    properties (Access = public)
        x (1, 1) double = 0; % x-coordinate [m]
        y (1, 1) double = 0; % y-coordinate [m]
        yaw (1, 1) double = 0; % yaw [rad]
        speed (1, 1) double = 0; % speed [m/s]
        steering (1, 1) double = 0; % steering [rad]
    end

    methods (Access = public)

        function obj = PlantMeasurement(x, y, yaw, speed, steering)

            % no-argument constructor
            if nargin == 0
                return
            end

            obj.x = x;
            obj.y = y;
            obj.yaw = yaw;
            obj.speed = speed;
            obj.steering = steering;
        end

    end

end
