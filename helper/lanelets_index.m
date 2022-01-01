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
    
    
    
%     nlanelets_i = [];
%     stop_flag = false;
% 
%     ref_points_index_i = reshape(iter.referenceTrajectoryIndex(i,:,:),Hp,1);
%     ref_points_i = reshape(iter.referenceTrajectoryPoints(i,:,:),Hp,2);
% 
%     for n = 1:length(ref_points_index_i)
%         nlanelets_i = [ nlanelets_i, sum(ref_points_index_i(n) > scenario.vehicles(1,i).points_index)+1];
%     end              
%     nlanelets_i = unique(nlanelets_i);
%     lanelets_index_i = scenario.vehicles(1,i).lanelets_index(nlanelets_i);
    
    
    
        
end     

