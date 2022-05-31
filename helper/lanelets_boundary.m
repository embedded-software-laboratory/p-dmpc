function lanelet_boundary = lanelets_boundary(scenario, iter)

% lanelets_boundary  return the lanelet boundaries for each vehicles
    
    nVeh = size(iter.referenceTrajectoryPoints,1);
    Hp = size(iter.referenceTrajectoryPoints,2);
    lanelets = scenario.lanelets;
    lanelet_boundary = cell(1,nVeh);

    
    for i = 1:nVeh
        % check the lanelet index based on the reference points
        nlanelets_i = [];     
        ref_points_index_i = reshape(iter.referenceTrajectoryIndex(i,:,:),1,Hp);
        
        for n = ref_points_index_i
            nlanelets_i = [ nlanelets_i, sum(n > scenario.vehicles(1,i).points_index)+1];
        end 


        lanelets_index = match_pose_to_lane(scenario, scenario.vehicles(1,i).x_position, scenario.vehicles(1,i).y_position);

        %lanelets_index = scenario.vehicles(1,i).lanelets_index(nlanelets_i);
        %[~, idx, ~] = unique(lanelets_index);
        %lanelets_index = lanelets_index(sort(idx));
        
        % determine the boundaries for each vehicle
        left_bound = [];
        right_bound = [];
        for index = lanelets_index
            left_bound = [left_bound; scenario.lanelet_boundary{1,index}{1,1}];
            right_bound = [right_bound;scenario.lanelet_boundary{1,index}{1,2}];
        end
        
        lanelet_boundary{i} = {left_bound,right_bound};

    end 

end