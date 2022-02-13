function is_leading_vehicle = check_driving_order(scenario,iter, nveh, iveh)
        
        is_leading_vehicle = false;
        current_point_nveh_x = iter.x0(nveh,1);
        current_point_nveh_y = iter.x0(nveh,2);
        current_point_nveh = [current_point_nveh_x,current_point_nveh_y];
        
        % reference trajectory points of adjacent vehicles
        current_point_iveh_x = iter.x0(iveh,1);
        current_point_iveh_y = iter.x0(iveh,2);
        current_point_iveh = [current_point_iveh_x,current_point_iveh_y];
        
        intersection1 = [20,21,78,69,70,77];
        intersection1_end = [scenario.lanelets{69}(end,LaneletInfo.lx),scenario.lanelets{69}(end,LaneletInfo.ly)];
        intersection2 = [18,25,19,24,46,47];
        intersection2_end = [scenario.lanelets{19}(end,LaneletInfo.lx),scenario.lanelets{19}(end,LaneletInfo.ly)];
        intersection3 = [44,51,43,52,98,99];
        intersection3_end = [scenario.lanelets{43}(end,LaneletInfo.lx),scenario.lanelets{43}(end,LaneletInfo.ly)];
        intersection4 = [96,103,97,102,72,73];
        intersection4_end = [scenario.lanelets{97}(end,LaneletInfo.lx),scenario.lanelets{97}(end,LaneletInfo.ly)];
        
        merging1 = [1,2,15,16,3,4,22];
        merging1_end = [scenario.lanelets{3}(end,LaneletInfo.lx),scenario.lanelets{3}(end,LaneletInfo.ly)];
        merging2 = [59,60,61,62,57,58,75];
        merging2_end = [scenario.lanelets{57}(end,LaneletInfo.lx),scenario.lanelets{57}(end,LaneletInfo.ly)];
        merging3 = [79,80,93,94,81,82,100];
        merging3_end = [scenario.lanelets{81}(end,LaneletInfo.lx),scenario.lanelets{81}(end,LaneletInfo.ly)];
        merging4 = [33,34,35,36,31,32,49];
        merging4_end = [scenario.lanelets{31}(end,LaneletInfo.lx),scenario.lanelets{31}(end,LaneletInfo.ly)];
        

%         % check the lanelet index for nveh
%         nlanelets_nveh = [];   
%         Hp = size(iter.referenceTrajectoryPoints,2);
%         ref_points_index_i = reshape(iter.referenceTrajectoryIndex(nveh,:,:),Hp,1);
%         
%         for n = 1:length(ref_points_index_i)
%             nlanelets_nveh = [ nlanelets_nveh, sum(ref_points_index_i(n) > scenario.vehicles(1,nveh).points_index)+1];
%         end              
%         nlanelets_nveh = unique(nlanelets_nveh);
%         lanelets_index_nveh = scenario.vehicles(1,nveh).lanelets_index(nlanelets_nveh); 
        lanelets_index_nveh = scenario.vehicles(1,nveh).predicted_lanelets;
       
%         % check the lanelet index for iveh
%         nlanelets_iveh = [];   
%         ref_points_index_i = reshape(iter.referenceTrajectoryIndex(iveh,:,:),Hp,1);
%         
%         for n = 1:length(ref_points_index_i)
%             nlanelets_iveh = [ nlanelets_iveh, sum(ref_points_index_i(n) > scenario.vehicles(1,iveh).points_index)+1];
%         end              
%         nlanelets_iveh = unique(nlanelets_iveh);
%         lanelets_index_iveh = scenario.vehicles(1,iveh).lanelets_index(nlanelets_iveh);
        lanelets_index_iveh = scenario.vehicles(1,iveh).predicted_lanelets;
        
        if sum(ismember(lanelets_index_nveh,intersection1)) && sum(ismember(lanelets_index_iveh,intersection1))
            %check the distance to the end of center of lanelets 69 and 78
            distance_nveh = norm(intersection1_end-current_point_nveh);
            distance_iveh = norm(intersection1_end-current_point_iveh);
            if distance_nveh < distance_iveh
                is_leading_vehicle = true;
            end
        elseif sum(ismember(lanelets_index_nveh,intersection2)) && sum(ismember(lanelets_index_iveh,intersection2))
            %check the distance to the end of center of lanelets 69 and 78
            distance_nveh = norm(intersection2_end-current_point_nveh);
            distance_iveh = norm(intersection2_end-current_point_iveh);
            if distance_nveh < distance_iveh
                is_leading_vehicle = true;
            end            
        elseif sum(ismember(lanelets_index_nveh,intersection3)) && sum(ismember(lanelets_index_iveh,intersection3))
            %check the distance to the end of center of lanelets 69 and 78
            distance_nveh = norm(intersection3_end-current_point_nveh);
            distance_iveh = norm(intersection3_end-current_point_iveh);
            if distance_nveh < distance_iveh
                is_leading_vehicle = true;
            end            
        elseif sum(ismember(lanelets_index_nveh,intersection4)) && sum(ismember(lanelets_index_iveh,intersection4))
            %check the distance to the end of center of lanelets 69 and 78
            distance_nveh = norm(intersection4_end-current_point_nveh);
            distance_iveh = norm(intersection4_end-current_point_iveh);
            if distance_nveh < distance_iveh
                is_leading_vehicle = true;
            end  
        elseif sum(ismember(lanelets_index_nveh,merging1)) && sum(ismember(lanelets_index_iveh,merging1))
            %check the distance to the end of center of lanelets 1 and 2
            distance_nveh = norm(merging1_end-current_point_nveh);
            distance_iveh = norm(merging1_end-current_point_iveh);
            if distance_nveh < distance_iveh
                is_leading_vehicle = true;
            end  
        elseif sum(ismember(lanelets_index_nveh,merging2)) && sum(ismember(lanelets_index_iveh,merging2))
            %check the distance to the end of center of lanelets 59 and 60
            distance_nveh = norm(merging2_end-current_point_nveh);
            distance_iveh = norm(merging2_end-current_point_iveh);
            if distance_nveh < distance_iveh
                is_leading_vehicle = true;
            end             
        elseif sum(ismember(lanelets_index_nveh,merging3)) && sum(ismember(lanelets_index_iveh,merging3))
            %check the distance to the end of center of lanelets 79 and 80
            distance_nveh = norm(merging3_end-current_point_nveh);
            distance_iveh = norm(merging3_end-current_point_iveh);
            if distance_nveh < distance_iveh
                is_leading_vehicle = true;
            end             
        elseif sum(ismember(lanelets_index_nveh,merging4)) && sum(ismember(lanelets_index_iveh,merging4))
            %check the distance to the end of center of lanelets 33 and 34
            distance_nveh = norm(merging4_end-current_point_nveh);
            distance_iveh = norm(merging4_end-current_point_iveh);
            if distance_nveh < distance_iveh
                is_leading_vehicle = true;
            end             
            
        else
            first_refPoint_nveh_x = iter.referenceTrajectoryPoints(nveh,1,1);
            first_refPoint_nveh_y = iter.referenceTrajectoryPoints(nveh,1,2);
            first_refPoint_nveh = [first_refPoint_nveh_x,first_refPoint_nveh_y];

            end_refPoint_nveh_x = iter.referenceTrajectoryPoints(nveh,end,1);
            end_refPoint_nveh_y = iter.referenceTrajectoryPoints(nveh,end,2);
            end_refPoint_nveh = [end_refPoint_nveh_x,end_refPoint_nveh_y];

    %         driving_direction = first_refPoint - current_point;
            driving_direction_nveh = end_refPoint_nveh - current_point_nveh;



            first_refPoint_iveh_x = iter.referenceTrajectoryPoints(iveh,1,1);
            first_refPoint_iveh_y = iter.referenceTrajectoryPoints(iveh,1,2);
            first_refPoint_iveh = [first_refPoint_iveh_x,first_refPoint_iveh_y];

            end_refPoint_iveh_x = iter.referenceTrajectoryPoints(iveh,end,1);
            end_refPoint_iveh_y = iter.referenceTrajectoryPoints(iveh,end,2);
            end_refPoint_iveh = [end_refPoint_iveh_x ,end_refPoint_iveh_y];

    %         driving_direction_adj = first_refPoint_adj - current_point_adj;
            driving_direction_iveh = end_refPoint_iveh - current_point_iveh;

            driving_direction_ref = (driving_direction_nveh + driving_direction_iveh)/2;
    %         driving_direction_ref = driving_direction;


            vehicle_relative_direction = current_point_nveh - current_point_iveh;
    %         vehicle_relative_direction = first_refPoint - first_refPoint_adj;

            % check if driving consecutively
            is_leading_vehicle = (dot(driving_direction_ref, vehicle_relative_direction) > 0);

        end
end












