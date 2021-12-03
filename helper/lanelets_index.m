function lanelets_idx = lanelets_index(veh_idx,iter,scenario)  
% lanelets_index returns the lanelets index based on their iteration reference
% trajectory points
    
    lanelets_idx = [];
    Hp = size(iter.referenceTrajectoryPoints,2);
    ref_points_index = reshape(iter.referenceTrajectoryIndex(veh_idx,:,:),1,Hp);

    % the lanelet index within prediction horizon
    for n = ref_points_index
        lanelets_idx = [ lanelets_idx, sum(n > scenario.vehicles(1,veh_idx).points_index)+1];
    end 
        
end           