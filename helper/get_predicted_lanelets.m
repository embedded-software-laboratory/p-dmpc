function [predicted_lanelets, reference, v_ref] = get_predicted_lanelets(scenario, iVeh, trim_current, x0, y0)
% GET_PREDICTED_LANELETS This function calculate the predicted lanelets
% based on vehile's current states and reference path. 
% 
% INPUT:
%   vehicle: instance of the class Vehicle
% 
%   trim_current: current trim
% 
%   x0: x-coordinate
% 
%   y0: y-coordinate
% 
%   mpa: calss MotionPrimitiveAutomaton
% 
%   dt: RHC sample time
% 
% OUTPUT:
%   predicted_lanelets: a row vector contains the predicted lanelets
% 
%   reference: ReferencePoints [x1 y1; x2 y2; ...] and corresponding
%   ReferenceIndex (point index)
% 
%   v_ref: reference speed

    if ((scenario.vehicle_ids(iVeh) == scenario.manual_vehicle_id) && scenario.manual_mpa_initialized) ...
        || ((scenario.vehicle_ids(iVeh) == scenario.second_manual_vehicle_id) && scenario.second_manual_mpa_initialized)
        mpa = scenario.vehicles(iVeh).vehicle_mpa;
    else
        mpa = scenario.mpa;
    end
 
    Hp = size(mpa.transition_matrix_single,3);

    % get reference speed and path points
    v_ref = get_max_speed(mpa, trim_current);
    
    % Find equidistant points on the reference trajectory.
    reference = sampleReferenceTrajectory(...
        Hp, ...                             % number of prediction steps
        scenario.vehicles(iVeh).referenceTrajectory, ...    % total reference path
        x0, ...                                             % vehicle position x
        y0, ...                                             % vehicle position y
        v_ref*scenario.dt, ...                                       % distance traveled in one timestep
        scenario.vehicles(iVeh).autoUpdatedPath, ...        % if the path has been updated automatically
        scenario.options.isParl, ...                        % parallel computation
        scenario.vehicles(iVeh).last_trajectory_index, ...  % last trajectory index of vehicle
        scenario.options.is_mixed_traffic...                % prevent loops in mixed traffic
    );

    ref_points_index = reshape(reference.ReferenceIndex,Hp,1);

    if strcmp(scenario.name,'Commonroad')
        predicted_lanelets_idx = [];

        for i_points_index = 1:length(ref_points_index)
            predicted_lanelets_idx = [predicted_lanelets_idx, sum(ref_points_index(i_points_index)>scenario.vehicles(iVeh).points_index)+1];
        end

        predicted_lanelets_idx = unique(predicted_lanelets_idx,'stable'); % use 'stable' to keep the order

        if scenario.options.isParl
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
