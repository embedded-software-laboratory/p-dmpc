function lanelet_boundary = get_lanelets_boundary(predicted_lanelets, lanelet_boundaries)
% GET_LANELETS_BOUNDARY Return the lanelet boundaries based on the
% predicted lanelets
    
    % determine the boundaries for each vehicle
    left_bound = [];
    right_bound = [];
    for index_lanelet = predicted_lanelets
        left_bound = [left_bound; lanelet_boundaries{index_lanelet}{1}];
        right_bound = [right_bound; lanelet_boundaries{index_lanelet}{2}];
    end
    
    lanelet_boundary = {left_bound,right_bound};

end