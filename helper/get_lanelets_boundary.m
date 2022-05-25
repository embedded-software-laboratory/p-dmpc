function lanelet_boundary = get_lanelets_boundary(predicted_lanelets, lanelet_boundaries)
% GET_LANELETS_BOUNDARY Return the lanelet boundaries based on the
% predicted lanelets
    
    % extract the target lanelets
    lanelet_boundaries = lanelet_boundaries(predicted_lanelets);

    % determine the boundaries for each vehicle
    left_bound = cellfun(@(c)c{1}', lanelet_boundaries, 'UniformOutput',false);
    left_bound = [left_bound{:}];

    right_bound = cellfun(@(c)c{2}', lanelet_boundaries, 'UniformOutput',false);
    right_bound = [right_bound{:}];
    
    lanelet_boundary = {left_bound,right_bound};

end


