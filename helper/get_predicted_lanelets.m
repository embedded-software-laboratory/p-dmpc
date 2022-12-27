function [predicted_lanelets, reference, v_ref, scenario] = get_predicted_lanelets(scenario, iVeh, x0, y0)
% GET_PREDICTED_LANELETS This function calculate the predicted lanelets
% based on vehile's current states and reference path. 
% 
% INPUT:
%   scenario: object of the class `Scenario`
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

    % use index based on pose, as vehicle in Expert-Mode has no defined trajectory
    if scenario.options.is_mixed_traffic && ...
        (((scenario.options.veh_ids(iVeh) == str2double(scenario.options.mixed_traffic_config.first_manual_vehicle_id) && scenario.options.mixed_traffic_config.first_manual_vehicle_mode == Control_Mode.Expert_mode) ...
        || (scenario.options.veh_ids(iVeh) == str2double(scenario.options.mixed_traffic_config.second_manual_vehicle_id) && scenario.options.mixed_traffic_config.second_manual_vehicle_mode == Control_Mode.Expert_mode)))
        predicted_lanelets = match_pose_to_lane(scenario, x0, y0);
        reference = [];
        v_ref = 0;
        return
    end

    if scenario.options.is_mixed_traffic && ...
        (((scenario.options.veh_ids(iVeh) == str2double(scenario.options.mixed_traffic_config.first_manual_vehicle_id)) && scenario.manual_mpa_initialized) ...
        || ((scenario.options.veh_ids(iVeh) == str2double(scenario.options.mixed_traffic_config.second_manual_vehicle_id)) && scenario.second_manual_mpa_initialized))
        mpa = scenario.vehicles(iVeh).vehicle_mpa;
    else
        mpa = scenario.mpa;
    end
 
    Hp = size(mpa.transition_matrix_single,3);

    % get reference speed and path points
    v_ref = get_max_speed_of_mpa(mpa);
    
    % Find equidistant points on the reference trajectory.
    [reference,curTrajectoryIndex] = sampleReferenceTrajectory(...
        Hp, ...                             % number of prediction steps
        scenario.vehicles(iVeh).referenceTrajectory, ...    % total reference path
        x0, ...                                             % vehicle position x
        y0, ...                                             % vehicle position y
        v_ref*scenario.options.dt, ...                                       % distance traveled in one timestep
        scenario.vehicles(iVeh).autoUpdatedPath, ...        % if the path has been updated automatically
        scenario.options.isPB, ...                        % parallel computation
        scenario.vehicles(iVeh).last_trajectory_index, ...  % last trajectory index of vehicle
        scenario.options.is_mixed_traffic...                % prevent loops in mixed traffic
    );

    ref_points_index = reshape(reference.ReferenceIndex,Hp,1);
%     ref_points_index = [curTrajectoryIndex;ref_points_index]; % add current index of vehicle on its trajectory to consider the current position of the vehicle 
    
    % predict several points more such that the predicted lanelets can cover all reachable set. Only in this way, the bounded reachable sets will not be cutoff at its front
    n_points_total = size(scenario.vehicles(iVeh).referenceTrajectory,1);
    index_add = ref_points_index(end) + 4;
    if index_add>n_points_total
        index_add = index_add - n_points_total;
    end
    ref_points_index = [ref_points_index;index_add];

    if ~isempty(scenario.lanelets)
        predicted_lanelets_idx = [];

        for i_points_index = 1:length(ref_points_index)
            predicted_lanelets_idx = [predicted_lanelets_idx, sum(ref_points_index(i_points_index)>scenario.vehicles(iVeh).points_index)+1];
        end

        predicted_lanelets_idx = unique(predicted_lanelets_idx,'stable'); % use 'stable' to keep the order

        if scenario.options.isPB
            % at least two lanelets needed to predicted if parallel computation is used
            if length(predicted_lanelets_idx) == 1
                % at least predict two lanelets to avoid that the endpoint of the
                % reference path being too close to the end of the predicted lanelets
                predicted_lanelets_idx = [predicted_lanelets_idx,predicted_lanelets_idx+1];
                if predicted_lanelets_idx(end) > length(scenario.vehicles(iVeh).lanelets_index)
                    % loop back to the first lanelet
                    predicted_lanelets_idx(end) = 1;
                end
            end
        end

        predicted_lanelets = scenario.vehicles(iVeh).lanelets_index(predicted_lanelets_idx);
    else
        predicted_lanelets = [];
    end

end
