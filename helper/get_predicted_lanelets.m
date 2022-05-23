function predicted_lanelets = get_predicted_lanelets(vehicle, ref_points_index, lanelets)
% GET_PREDICTED_LANELETS This function calculate the predicted lanelets
% based on vehile's current states and reference path. 
% 
% INPUT:
%   vehicle: instance of the class Vehicle
% 
%   ref_points_index: indices points on vehicle's reference path
% 
%   lanelets: road raw data, where the information about oen lanelet's
%   predecessor and successor lanelets will be needed
% 
% OUTPUT:
%   predicted_lanelets: a row vector contains the predicted lanelets
% 
 
    predicted_lanelets_idx = [];  

    length_remaining = vehicle.Lf;
    last_ref_point_idx = ref_points_index(end);

    % extend the reference points to consider the length of the vehicle
    while length_remaining>0
        last_ref_point_before_extend = vehicle.referenceTrajectory(last_ref_point_idx,:);
        
        last_ref_point_idx = last_ref_point_idx + 1; % extend one point ahead
        if last_ref_point_idx > size(vehicle.referenceTrajectory,1)
            last_ref_point_idx = 1; % loop back
        end
        
        last_ref_point_after_extend = vehicle.referenceTrajectory(last_ref_point_idx,:);
        length_remaining = length_remaining - norm(last_ref_point_before_extend-last_ref_point_after_extend);
    end

    ref_points_index_extended = [ref_points_index;last_ref_point_idx];


    for i_points_index = 1:length(ref_points_index_extended)
        predicted_lanelets_idx = [predicted_lanelets_idx, sum(ref_points_index_extended(i_points_index)>vehicle.points_index)+1];
    end
    
    predicted_lanelets_idx = unique(predicted_lanelets_idx);
    
    predicted_lanelets = vehicle.lanelets_index(predicted_lanelets_idx);

    % reorder the predicted lanelets such that they are always in the order
    % of predecessor lanelet -> sucessor lanelet
    if length(predicted_lanelets)>1
        % reorder is only needed if there are more than one predicted lanelets
        predicted_lanelets_reordered = zeros(1,0);
        % assume the first lanelet is indeed the first lanelet
        predicted_lanelets_reordered(1) = predicted_lanelets(1);
    
        is_find_successors_finished = false;
        is_find_predecessors_finished = false;

        while ~is_find_successors_finished || ~is_find_predecessors_finished
            % find successor lanelet
            if ~is_find_successors_finished
                last_predicted_lanelet = predicted_lanelets_reordered(end);
                successors = [lanelets(last_predicted_lanelet).successor.refAttribute];
                find_successor = find(ismember(predicted_lanelets,successors));
                if ~isempty(find_successor)
                    predicted_lanelets_reordered = [predicted_lanelets_reordered, predicted_lanelets(find_successor)];
                else
                    is_find_successors_finished = true;
                end
            end

            % find predecessor lanelet
            if ~is_find_predecessors_finished
                first_lanelet = predicted_lanelets_reordered(1);
                predecessors = [lanelets(first_lanelet).predecessor.refAttribute];
                find_predecessor = find(ismember(predicted_lanelets,predecessors));
                if ~isempty(find_predecessor)
                    predicted_lanelets_reordered = [predicted_lanelets(find_predecessor), predicted_lanelets_reordered];
                else
                    is_find_predecessors_finished = true;
                end
            end

            if length(predicted_lanelets_reordered)==length(predicted_lanelets)
                is_find_successors_finished = true;
                is_find_predecessors_finished = true;
            end
        end
        predicted_lanelets = predicted_lanelets_reordered; % output
    end

end