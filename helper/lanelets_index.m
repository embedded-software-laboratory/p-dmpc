function lanelets_idx = lanelets_index(veh_idx,iter,scenario)  
% lanelets_index returns the lanelets index based on their iteration reference
% trajectory points
    
    nlanelets_idx = [];
    ref_points_index = iter.referenceTrajectoryIndex(veh_idx,:);

    % the lanelet index within prediction horizon
    for n = ref_points_index
        nlanelets_idx = [ nlanelets_idx, sum(n > scenario.vehicles(1,veh_idx).points_index)+1];
    end 
    
    lanelets_idx = scenario.vehicles(1,veh_idx).lanelets_index(nlanelets_idx);
        
end           