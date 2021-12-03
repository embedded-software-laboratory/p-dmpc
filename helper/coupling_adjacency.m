function adjacency = coupling_adjacency(scenario, iter)
% COUPLING_ADJACENCY returns the adjacency matrix based on predicted
% trajectories. If the future trajectories are within  the specified
% distance, the two vehicles are considered as adjacent.
    
    nVeh = size(iter.referenceTrajectoryPoints,1);
    Hp = size(iter.referenceTrajectoryPoints,2);
    adjacency = zeros(nVeh,nVeh);
    [~,adjacent,~] = commonroad_lanelets();
%     save('adjacent.mat','adjacent')
    
    for i = 1:nVeh-1
        nlanelets_i = [];
        stop_flag = false;
        
        ref_points_index_i = reshape(iter.referenceTrajectoryIndex(i,:,:),1,Hp);
        
        for n = ref_points_index_i
            nlanelets_i = [ nlanelets_i, sum(n > scenario.vehicles(1,i).points_index)+1];
        end 
%         disp(nlanelets_i)
        lanelets_index_i = scenario.vehicles(1,i).lanelets_index(nlanelets_i);
        
        
        for j = nVeh:-1:i+1
            nlanelets_j = [];
            ref_points_index_j = reshape(iter.referenceTrajectoryIndex(j,:,:),1,Hp);
            for n = ref_points_index_j
                nlanelets_j = [ nlanelets_j, sum(n > scenario.vehicles(1,j).points_index)+1];
            end 
%             disp(nlanelets_j)
            lanelets_index_j = scenario.vehicles(1,j).lanelets_index(nlanelets_j);

            for k_i = 1:length(lanelets_index_i)
                
                for k_j = 1:length(lanelets_index_j)
                    
                    if adjacent(lanelets_index_i(k_i),lanelets_index_j(k_j)) == 1
                        adjacency(i,j) = 1;
                        adjacency(j,i) = 1;
                        stop_flag = true;
                    end

                    if stop_flag
                        break
                    end
                end

            end   
        
        end

    end
    
    
 
    
%     nVeh = size(iter.referenceTrajectoryPoints,1);
%     Hp = size(iter.referenceTrajectoryPoints,2);
%     adjacency = zeros(nVeh,nVeh);
% 
%     for i = 1:nVeh-1
%         
%         stop_flag = false;
%         ref_points_i = reshape(iter.referenceTrajectoryPoints(i,:,:),2,Hp);
%         
%         for j = nVeh:-1:i+1     
%             ref_points_j = reshape(iter.referenceTrajectoryPoints(j,:,:),2,Hp);
% 
%             for k = 1:Hp
% 
%                 dist = norm((ref_points_i(:,k)-ref_points_j(:,k)),2);
% 
%                 if dist < 1
%                     adjacency(i,j) = 1;
%                     adjacency(j,i) = 1;
%                     stop_flag = true;
%                 end
%                 
%                 if stop_flag
%                     break
%                 end
% 
%             end   
%         
%         end

%     end




end