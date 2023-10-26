function [reference_path_struct, v_ref] = get_reference_trajectory(mpa, total_reference_path, x0, y0, trim, dt_seconds)
    % get_reference_trajectory This function calculates the reference path and
    % speed for the prediction horizon based on the vehicle's current state and
    % the total reference path from the scenario
    %
    % OUTPUT:
    %   reference_path_struct struct with fields
    %       path (Hp, 2) [x1 y1; x2 y2; ...]
    %       ReferenceIndex (Hp, 1) point indices
    %
    %   v_ref (Hp, 1) reference speed

    arguments
        mpa MotionPrimitiveAutomaton
        total_reference_path (:, 2) double % total_reference_path from scenario
        x0 (1, 1) double % current x coordinate of the vehicle
        y0 (1, 1) double % current y coordinate of the vehicle
        trim (1, 1) double % current trim of the vehicle
        dt_seconds (1, 1) double % step time
    end

    Hp = size(mpa.transition_matrix_single, 3);

    % get reference speed and path points
    v_ref = mpa.get_max_speed(trim);

    % determine intermediate speed between every two consecutive speeds
    v_current = mpa.trims(trim).speed;
    v_ref_intermediate = ([v_current; v_ref(1:end - 1)] + v_ref) / 2;

    % distance that can be traveled in each step of Hp
    distance_max = v_ref_intermediate * dt_seconds;

    % Find equidistant points on the reference path.
    reference_path_struct = sample_reference_trajectory( ...
        Hp, ... % number of prediction steps
        total_reference_path, ... % total_reference_path
        x0, ... % vehicle position x
        y0, ... % vehicle position y
        distance_max ... % distance traveled in one time step
    );

end
