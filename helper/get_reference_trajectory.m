function [reference, v_ref] = get_reference_trajectory(scenario, mpa, iter, iVeh, x0, y0)
    % get_reference_trajectory This function calculate the reference path and
    % speed for the prediction horizon based on the vehicle's current state and
    % the reference path from the scenario
    %
    % OUTPUT:
    %   reference struct with fields
    %       ReferencePoints (Hp, 2) [x1 y1; x2 y2; ...]
    %       ReferenceIndex (Hp, 1) point indices
    %
    %   v_ref (Hp, 1) reference speed

    arguments
        scenario Scenario
        mpa MotionPrimitiveAutomaton
        iter IterationData
        iVeh (1, 1) double % index of the vehicle in the vehicle list
        x0 (1, 1) double % current x coordinate of the vehicle
        y0 (1, 1) double % current y coordinate of the vehicle
    end

    Hp = size(mpa.transition_matrix_single, 3);

    % get reference speed and path points
    v_ref = mpa.get_max_speed(iter.trim_indices(iVeh));

    % determine intermediate speed between every two consecutive speeds
    v_current = mpa.trims(iter.trim_indices(iVeh)).speed;
    v_ref_intermediate = ([v_current; v_ref(1:end - 1)] + v_ref) / 2;

    % distance that can be traveled in each step of Hp
    distance_max = v_ref_intermediate * scenario.options.dt_seconds;

    % Find equidistant points on the reference trajectory.
    reference = sample_reference_trajectory( ...
        Hp, ... % number of prediction steps
        scenario.vehicles(iVeh).reference_path, ... % total reference path
        x0, ... % vehicle position x
        y0, ... % vehicle position y
        distance_max ... % distance traveled in one time step
    );

end
