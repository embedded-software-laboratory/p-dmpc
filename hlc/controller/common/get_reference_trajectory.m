function [reference_trajectory_struct, v_ref, current_point_index] = get_reference_trajectory(mpa, reference_path, reference_speed, x_current, y_current, trim_current, dt_seconds)
    % get_reference_trajectory This function calculates the reference path and
    % speed for the prediction horizon based on the vehicle's current state and
    % the total reference path from the scenario. As a byproduct it returns
    % the index of the point on the curve closest to the given point (x_current, y_current)
    %
    % OUTPUT:
    %   reference_trajectory_struct struct with fields
    %       path (Hp, 2) [x1 y1; x2 y2; ...]
    %       points_index (Hp, 1) point indices
    %
    %   v_ref (Hp, 1) reference speed
    %
    %   current_point_index Index of point on the curve
    %       closest to given point (x_current, y_current)

    arguments
        mpa MotionPrimitiveAutomaton
        reference_path (:, 2) double % reference_path from scenario
        reference_speed (1, 1) double % reference_speed from scenario
        x_current (1, 1) double % current x coordinate of the vehicle
        y_current (1, 1) double % current y coordinate of the vehicle
        trim_current (1, 1) double % current trim of the vehicle
        dt_seconds (1, 1) double % step time
    end

    Hp = size(mpa.transition_matrix_single, 3);

    % get reference speed and path points
    v_ref = ones(Hp, 1) * reference_speed;

    % determine intermediate speed between every two consecutive speeds
    v_current = mpa.trims(trim_current).speed;
    v_ref_intermediate = ([v_current; v_ref(1:end - 1)] + v_ref) / 2;

    % distance that can be traveled in each step of Hp
    step_distances = v_ref_intermediate * dt_seconds;

    % Find equidistant points on the reference path.
    [reference_trajectory_struct, current_point_index] = sample_reference_trajectory( ...
        Hp, ... % number of prediction steps
        reference_path, ... % reference_path that is sampled including dt_seconds
        x_current, ... % current x coordinate of the vehicle
        y_current, ... % current y coordinate of the vehicle
        step_distances ... % distance traveled in one time step
    );

end
