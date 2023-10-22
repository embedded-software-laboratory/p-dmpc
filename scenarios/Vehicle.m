classdef Vehicle < handle
    % VEHICLE   Class representing vehicles in a scenario

    properties
        trim_config = 1; % initial trim configuration
        x_start = 0; % [m]
        y_start = 0; % [m]
        yaw_start = 0; % [radians]
        reference_path (:, 2) double; % [x1 y1; x2 y2; ...]
        Length = 0.22; % Vehicle length (bumper to bumper)[m]
        Width = 0.1; % Vehicle width [m]
        Lf = 0.1; % Distance between vehicle center and front axle center [m]
        Lr = 0.1; % Distance between vehicle center and rear axle center [m]
        ID = -1; % vehicle ID (should be positive integer)
        lanelets_index; % index of lanelets of the reference trajectory in the scenario lanelets
        points_index; % index of the last reference point of each lanelet from the reference trajectory
        communicate; % instance of the class `Communication`, used for communication via ROS 2
        reference_path_loop_idx % totally 7 loops of paths are designed
        is_loop = true; % whether the reference path is a loop
    end

    methods

        function obj = Vehicle()

        end

        function occupied_areas = get_occupied_areas(obj, x, y, yaw, offset)
            % the function calculates the local occupied area based on the
            % dimensions of the vehicle and translates it to the global frame
            %
            % Output:
            %   occupied_areas (1, 1) struct with the fields
            %       normal_offset (2, :) double [x; y]
            %       without_offset (2, :) double [x; y]

            arguments
                obj Vehicle
                x (1, 1) double % current x coordinate
                y (1, 1) double % current y coordinate
                yaw (1, 1) double % current yaw
                offset (1, 1) double % offset for the occupied area
            end

            % get vehicles currently occupied area with offset
            % repeat the first entry to enclose the shape
            x_rec1 = [-1, -1, 1, 1, -1] * (obj.Length / 2 + offset);
            y_rec1 = [-1, 1, 1, -1, -1] * (obj.Width / 2 + offset);
            [x_rec2, y_rec2] = translate_global(yaw, x, y, x_rec1, y_rec1);
            occupied_areas.normal_offset = [x_rec2; y_rec2];

            % get vehicles currently occupied area without offset
            % repeat the first entry to enclose the shape
            x_rec1_without_offset = [-1, -1, 1, 1, -1] * (obj.Length / 2);
            y_rec1_without_offset = [-1, 1, 1, -1, -1] * (obj.Width / 2);
            [x_rec2_without_offset, y_rec2_without_offset] = translate_global(yaw, x, y, x_rec1_without_offset, y_rec1_without_offset);
            occupied_areas.without_offset = [x_rec2_without_offset; y_rec2_without_offset];

        end

        function plot(obj, color)
            vehicle_polygon = transformed_rectangle(obj.x_start, obj.y_start, obj.yaw_start, obj.Length, obj.Width);
            patch(vehicle_polygon(1, :), vehicle_polygon(2, :), color);
        end

    end

end
