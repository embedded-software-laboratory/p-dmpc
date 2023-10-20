function [predicted_lanelets, reference, v_ref] = get_predicted_lanelets(scenario, mpa, iter, iVeh, x0, y0)
    % GET_PREDICTED_LANELETS This function calculate the predicted lanelets
    % based on vehile's current states and reference path.
    %
    % INPUT:
    %   scenario: object of the class `Scenario`
    %
    %   mpa: object of the class `MotionPrimitiveAutomaton`
    %
    %   iVeh: index of vehicle
    %
    %   trim_current: current trim
    %
    %   x0: x-coordinate
    %
    %   y0: y-coordinate
    %
    % OUTPUT:
    %   predicted_lanelets: a row vector contains the predicted lanelets
    %
    %   reference: ReferencePoints [x1 y1; x2 y2; ...] and corresponding
    %   ReferenceIndex (point index)
    %
    %   v_ref: reference speed

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

    if scenario.options.scenario_type == ScenarioType.circle
        predicted_lanelets = [];
        return
    end

    ref_points_index = reshape(reference.ReferenceIndex, Hp, 1);
    %     ref_points_index = [curTrajectoryIndex;ref_points_index]; % add current index of vehicle on its trajectory to consider the current position of the vehicle

    % predict several points more such that the predicted lanelets can cover all reachable set. Only in this way, the bounded reachable sets will not be cutoff at its front
    n_points_total = size(scenario.vehicles(iVeh).reference_path, 1);
    index_add = ref_points_index(end) + 4;

    if index_add > n_points_total
        index_add = index_add - n_points_total;
    end

    ref_points_index = [ref_points_index; index_add];

    predicted_lanelets_idx = zeros(length(ref_points_index), 1);

    for i_points_index = 1:length(ref_points_index)
        predicted_lanelets_idx(i_points_index) = sum(ref_points_index(i_points_index) > scenario.vehicles(iVeh).points_index) + 1;
    end

    predicted_lanelets_idx = unique(predicted_lanelets_idx, 'stable'); % use 'stable' to keep the order

    if scenario.options.is_prioritized
        % at least two lanelets needed to predicted if parallel computation is used
        if length(predicted_lanelets_idx) == 1
            % at least predict two lanelets to avoid that the endpoint of the
            % reference path being too close to the end of the predicted lanelets
            predicted_lanelets_idx = [predicted_lanelets_idx, predicted_lanelets_idx + 1];

            if predicted_lanelets_idx(end) > length(scenario.vehicles(iVeh).lanelets_index)
                % loop back to the first lanelet
                predicted_lanelets_idx(end) = 1;
            end

        end

    end

    predicted_lanelets = scenario.vehicles(iVeh).lanelets_index(predicted_lanelets_idx);

end
