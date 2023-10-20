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

    [reference, v_ref] = get_reference_trajectory(scenario, mpa, iter, iVeh, x0, y0);

    if scenario.options.scenario_type == ScenarioType.circle
        predicted_lanelets = [];
        return
    end

    ref_points_index = reference.ReferenceIndex;
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

    if length(predicted_lanelets_idx) == 1
        % at least predict two lanelets to avoid that the endpoint of the
        % reference path being too close to the end of the predicted lanelets
        predicted_lanelets_idx = [predicted_lanelets_idx, predicted_lanelets_idx + 1];

        if predicted_lanelets_idx(end) > length(scenario.vehicles(iVeh).lanelets_index)
            % loop back to the first lanelet
            predicted_lanelets_idx(end) = 1;
        end

    end

    predicted_lanelets = scenario.vehicles(iVeh).lanelets_index(predicted_lanelets_idx);

end
