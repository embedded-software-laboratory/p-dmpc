classdef Vehicle < handle
    % VEHICLE   Class representing vehicles in a scenario

    properties
        trim_config = 1; % initial trim configuration
        x_start = 0; % [m]
        y_start = 0; % [m]
        yaw_start = 0; % [radians]
        x_goal = 0; % [m]
        y_goal = 0; % [m]
        yaw_goal = 0; % [radians]
        reference_trajectory = [0 0; 1 0; 3 1]; % [x1 y1; x2 y2; ...]
        Length = 0.22; % Vehicle length (bumper to bumper)[m]
        Width = 0.1; % Vehicle width [m]
        Lf = 0.1; % Distance between vehicle center and front axle center [m]
        Lr = 0.1; % Distance between vehicle center and rear axle center [m]
        ID = -1; % vehicle ID (should be positive integer)
        lanelets_index;
        points_index;
        communicate; % instance of the class `Communication`, used for communication via ROS 2
        vehicle_mpa; % instance of the class `MotionPrimitiveAutomaton`
        ref_path_loop_idx % totally 7 loops of paths are designed
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
