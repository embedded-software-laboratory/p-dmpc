function lanelet_boundary = lanelets_boundary(scenario, iter)

% lanelets_boundary  update the boundary for each vehicle based on predicted trajectories. 
    
    nVeh = size(iter.referenceTrajectoryPoints,1);
    Hp = size(iter.referenceTrajectoryPoints,2);
    lanelets = scenario.lanelets;
    lanelet_boundary = {};

    
    for i = 1:nVeh
        nlanelets_i = [];     
        ref_points_index_i = reshape(iter.referenceTrajectoryIndex(i,:,:),1,Hp);
        
        for n = ref_points_index_i
            nlanelets_i = [ nlanelets_i, sum(n > scenario.vehicles(1,i).points_index)+1];
        end 

        lanelets_index = scenario.vehicles(1,i).lanelets_index(nlanelets_i);
        [~, idx, ~] = unique(lanelets_index);
        lanelets_index = lanelets_index(sort(idx));
        
        boundary = [];
        
        for index = lanelets_index
            
            bound = [];     
            adjacentLeft = scenario.commonroad_data.lanelet(index).adjacentLeft;
            adjacentRight = scenario.commonroad_data.lanelet(index).adjacentRight;

            % if the lanelet has adjacent left lanelet, the boundary
            % should be the leftBound of adjacentLeft and rightBound of
            % the current lanelet
            if isfield(adjacentLeft,'refAttribute')
                adjacentLeft_index = horzcat(adjacentLeft.refAttribute);
                left_bound_x = lanelets{ adjacentLeft_index }(:,LaneletInfo.lx);
                left_bound_y = lanelets{ adjacentLeft_index }(:,LaneletInfo.ly);
                left_bound = [left_bound_x(1:end),left_bound_y(1:end)];

                right_bound_x = lanelets{ index }(:,LaneletInfo.rx);
                right_bound_y = lanelets{ index }(:,LaneletInfo.ry);
                right_bound = [right_bound_x(1:end),right_bound_y(1:end)];                           
            end

            if isfield(adjacentRight,'refAttribute')
                adjacentRight_index = horzcat(adjacentRight.refAttribute);
                left_bound_x = lanelets{ index }(:,LaneletInfo.lx);
                left_bound_y = lanelets{ index }(:,LaneletInfo.ly);
                left_bound = [left_bound_x(1:end),left_bound_y(1:end)];

                right_bound_x = lanelets{ adjacentRight_index }(:,LaneletInfo.rx);
                right_bound_y = lanelets{ adjacentRight_index }(:,LaneletInfo.ry);
                right_bound = [right_bound_x(1:end),right_bound_y(1:end)];
            end
            bound = [left_bound,right_bound];

            boundary = [boundary; bound];
        end
        
        lanelet_boundary{i} = boundary;
%         disp('lanelet_boudnary')
%         disp(lanelet_boundary(1,i))

    end 
    
            

end