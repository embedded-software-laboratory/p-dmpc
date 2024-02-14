classdef Vehicle
    % VEHICLE   Class representing vehicles in a scenario

    properties
        x_start = 0; % [m]
        y_start = 0; % [m]
        yaw_start = 0; % [radians]
        reference_path (:, 2) double; % [x1 y1; x2 y2; ...]
        reference_speed (1, 1) double = 0; % [m/s]
        Length = 0.22; % Vehicle length (bumper to bumper)[m]
        Width = 0.1; % Vehicle width [m]
        Lf = 0.1; % Distance between vehicle center and front axle center [m]
        Lr = 0.1; % Distance between vehicle center and rear axle center [m]
        lanelets_index; % index of lanelets of the reference path in the scenario lanelets
        points_index; % index of the last reference point of each lanelet from the reference path
        is_loop = true; % whether the reference path is a loop
    end

    methods

        function obj = Vehicle()

        end

        function plot(obj, color)
            vehicle_polygon = transformed_rectangle(obj.x_start, obj.y_start, obj.yaw_start, obj.Length, obj.Width);
            patch(vehicle_polygon(1, :), vehicle_polygon(2, :), color);
        end

    end

end
