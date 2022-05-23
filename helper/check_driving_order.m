function is_leading_vehicle = check_driving_order(scenario,iter, nveh, iveh)
% CHECK_DRIVING_ORDER
% 1. for vehicles merging into one lanelet on the highway or at the
% intersection, the driving order is determined by cheking the distance to
% the reference point; 2. for other cases, a general method of using the
% dot product is used to determine the driving order

% Return: is_leading_vehicle  --> true if nveh is driving ahead of iveh;
%                                 false otherwise
        
        is_leading_vehicle = false;
        current_point_nveh_x = iter.x0(nveh,1);
        current_point_nveh_y = iter.x0(nveh,2);
        current_point_nveh = [current_point_nveh_x,current_point_nveh_y];
        
        % reference trajectory points of adjacent vehicles
        current_point_iveh_x = iter.x0(iveh,1);
        current_point_iveh_y = iter.x0(iveh,2);
        current_point_iveh = [current_point_iveh_x,current_point_iveh_y];
        
        % define the reference point for checking driving order
        intersection1 = [20,21,78,69,104,95,70,77];
        intersection1_end = [scenario.lanelets{69}(end,LaneletInfo.rx),scenario.lanelets{69}(end,LaneletInfo.ry)];
        intersection2 = [18,25,19,24,71,76,46,47];
        intersection2_end = [scenario.lanelets{19}(end,LaneletInfo.rx),scenario.lanelets{19}(end,LaneletInfo.ry)];
        intersection3 = [44,51,43,52,17,26,98,99];
        intersection3_end = [scenario.lanelets{43}(end,LaneletInfo.rx),scenario.lanelets{43}(end,LaneletInfo.ry)];
        intersection4 = [96,103,97,102,45,50,72,73];
        intersection4_end = [scenario.lanelets{97}(end,LaneletInfo.rx),scenario.lanelets{97}(end,LaneletInfo.ry)];
        
        merging1 = [1,2,15,16,3,4,22];
        merging1_end = [scenario.lanelets{4}(end,LaneletInfo.cx),scenario.lanelets{4}(end,LaneletInfo.cy)];
        merging2 = [59,60,61,62,57,58,75];
        merging2_end = [scenario.lanelets{58}(end,LaneletInfo.cx),scenario.lanelets{58}(end,LaneletInfo.cy)];
        merging3 = [79,80,93,94,81,82,100];
        merging3_end = [scenario.lanelets{82}(end,LaneletInfo.cx),scenario.lanelets{82}(end,LaneletInfo.cy)];
        merging4 = [33,34,35,36,31,32,49];
        merging4_end = [scenario.lanelets{32}(end,LaneletInfo.cx),scenario.lanelets{32}(end,LaneletInfo.cy)];
        

        lanelets_index_nveh = iter.predicted_lanelets{nveh};       
        lanelets_index_iveh = iter.predicted_lanelets{iveh};
        
        
        if sum(ismember(lanelets_index_nveh,intersection1))>0 && sum(ismember(lanelets_index_iveh,intersection1))>0
            %check the distance to the end of center of lanelets 69 and 78
            distance_nveh = norm(intersection1_end-current_point_nveh);
            distance_iveh = norm(intersection1_end-current_point_iveh);
            if distance_nveh < distance_iveh
                is_leading_vehicle = true;
            end
        elseif sum(ismember(lanelets_index_nveh,intersection2))>0 && sum(ismember(lanelets_index_iveh,intersection2))>0
            %check the distance to the end of center of lanelets 69 and 78
            distance_nveh = norm(intersection2_end-current_point_nveh);
            distance_iveh = norm(intersection2_end-current_point_iveh);
            if distance_nveh < distance_iveh
                is_leading_vehicle = true;
            end            
        elseif sum(ismember(lanelets_index_nveh,intersection3))>0 && sum(ismember(lanelets_index_iveh,intersection3))>0
            %check the distance to the end of center of lanelets 69 and 78
            distance_nveh = norm(intersection3_end-current_point_nveh);
            distance_iveh = norm(intersection3_end-current_point_iveh);
            if distance_nveh < distance_iveh
                is_leading_vehicle = true;
            end            
        elseif sum(ismember(lanelets_index_nveh,intersection4))>0 && sum(ismember(lanelets_index_iveh,intersection4))>0
            %check the distance to the end of center of lanelets 69 and 78
            distance_nveh = norm(intersection4_end-current_point_nveh);
            distance_iveh = norm(intersection4_end-current_point_iveh);
            if distance_nveh < distance_iveh
                is_leading_vehicle = true;
            end  
        elseif sum(ismember(lanelets_index_nveh,merging1))>0 && sum(ismember(lanelets_index_iveh,merging1))>0
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

            % check if driving ahead
            is_leading_vehicle = (dot(driving_direction_ref, vehicle_relative_direction) > 0);

        end
end












