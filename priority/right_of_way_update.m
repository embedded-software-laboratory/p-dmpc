classdef  right_of_way_update < interface_priority
% right_of_way  Instance of interface_priority used for dynamic priority
% assignment. For vehicles driving consectively, vehicles in the front are
% aasigned with higher priorities; For vehicles at the intersection, vehicles
% closer to the center of intersection are assigned with higher priorities. Vehicles at intersection keep
% higher priority and do not change their relative priority until they leave intersection
    
    properties (Access=private)
        iter
    end
    
    methods 
        
        function obj = right_of_way_update(scenario,iter)
            obj.scenario = scenario;
            obj.iter = iter;
        end
        
        function [veh_at_intersection,groups] = priority(obj,last_veh_at_intersection)

            groups = struct;
            nVeh = length(obj.scenario.vehicles);
            Hp = size(obj.iter.referenceTrajectoryPoints,2);
            intersection_center = [2.25, 2];
            prio = 1;
            priority = ones(1,nVeh); 
            lanelets_idx = zeros(nVeh, Hp);
            Veh = 1:nVeh;
            veh_at_intersection = [];
            
            %% identify vehicles at intersection
            for veh_idx = 1:nVeh   
                lanelets_idx(veh_idx,:) = lanelets_index(veh_idx,obj.iter,obj.scenario);
                is_at_intersection = (sum(ismember(lanelets_idx(veh_idx,:), obj.scenario.intersection_lanelets)) > 0);
                if is_at_intersection
                    veh_at_intersection = [veh_at_intersection,veh_idx];
                    n_veh_at_intersection = length(veh_at_intersection);
                end
 
            end
            disp(['veh_at_intersection: ', num2str(veh_at_intersection)])
     
            % assign priorities to vehicles at the intersection
            if ~isempty(veh_at_intersection)
                
                if isempty(last_veh_at_intersection)
                    disp('come on !!!!!!!!!!!!')
                    first_referenceTrajectoryPoints = [obj.iter.referenceTrajectoryPoints(veh_at_intersection,1,1),obj.iter.referenceTrajectoryPoints(veh_at_intersection,1,2)];
                    diff = first_referenceTrajectoryPoints - intersection_center;
                    distance_to_center = sqrt(diff(:,1).^2 + diff(:,2).^2);
                    [~, index] = sort(distance_to_center,'ascend');

                    % sort the vehicles at intersection based on their distance to the intersection center
                    % and assign higher priority to the vehicle closer to the intersection center
                    veh_at_intersection = veh_at_intersection(index);
                    priority(veh_at_intersection)= 1:n_veh_at_intersection;

                    % highest priority assigned to vehicles that are not at intersection
                    prio = n_veh_at_intersection + 1;
                else
                    disp(['last_veh_at_intersection: ', num2str(last_veh_at_intersection)])
                    % all remained vehicles at intersection keep higher priorities, and then assign priorities to new vehicles
                    % driving into intersection
                    [~, idx,~] = intersect(last_veh_at_intersection,veh_at_intersection);
                    remained_veh_at_intersection = last_veh_at_intersection(sort(idx));
                    n_remained_veh_at_intersection = length(remained_veh_at_intersection);
                    priority(remained_veh_at_intersection) = 1:n_remained_veh_at_intersection;
                    disp(['remained_veh_at_intersection: ', num2str(remained_veh_at_intersection)])
                    disp(['priority: ',num2str(priority)])

                    % new incoming vehicles at intersection
                    new_veh_at_intersection = setdiff(veh_at_intersection, remained_veh_at_intersection);

                    % assign priorities to new vehicles at intersection
                    if ~isempty(new_veh_at_intersection)
                        n_new_veh_at_intersection = length(new_veh_at_intersection);
                        first_referenceTrajectoryPoints = [obj.iter.referenceTrajectoryPoints(new_veh_at_intersection,1,1),obj.iter.referenceTrajectoryPoints(new_veh_at_intersection,1,2)];
                        diff = first_referenceTrajectoryPoints - intersection_center;
                        distance_to_center = sqrt(diff(:,1).^2 + diff(:,2).^2);
                        [~, index] = sort(distance_to_center,'ascend');

                        % sort the vehicles at intersection based on their distance to the intersection center
                        % and assign higher priority to the vehicle closer to the intersection center
                        new_veh_at_intersection = new_veh_at_intersection(index);
                        priority(new_veh_at_intersection) = (n_remained_veh_at_intersection+1) : (n_remained_veh_at_intersection + n_new_veh_at_intersection);

                        % highest priority assigned to vehicles that are not at intersection      
                    end
                    disp(['priority: ',num2str(priority)])
                    prio = n_veh_at_intersection + 1; 
                    veh_at_intersection = [remained_veh_at_intersection,new_veh_at_intersection];% ordered veh_at_intersection
                end
                
            end
            
            %%
            % update the adjacency matrix for vehicles not at the intersection
            adjacency= obj.scenario.adjacency(:,:,end);         

            
            % update the undirected adjacency matrix to a directed adajacency matrix
            veh_not_at_intersection = setdiff(Veh,veh_at_intersection);
            nveh_not_at_intersection = length(veh_not_at_intersection);
            
            for nveh = 1: nVeh-1
                if ismember(nveh, veh_at_intersection)
                    continue
                end
                
                veh_adjacent = find(adjacency(nveh,:));
                veh_adjacent_not_at_intersection = setdiff(veh_adjacent, veh_at_intersection);
                %check the vehicles whose index is larger than the current
                %vehicle, no repeated check
                veh_adjacent_not_at_intersection = veh_adjacent_not_at_intersection(veh_adjacent_not_at_intersection > nveh);
                for iveh = veh_adjacent_not_at_intersection
                    
                    is_leading_vehicle = check_driving_order(obj.iter, nveh, iveh);
                    if is_leading_vehicle
                        adjacency(nveh, iveh) = 1;
                        adjacency(iveh, nveh) = 0;
                    else
                        adjacency(nveh, iveh) = 0;
                        adjacency(iveh, nveh) = 1;
                        
                    end
                    
                end
            
            end
            
            directed_adjacency_not_at_intersection = adjacency;            
            directed_adjacency_not_at_intersection(veh_at_intersection,:) = [];        
            directed_adjacency_not_at_intersection(:,veh_at_intersection) = [];
%             disp(directed_adjacency_not_at_intersection)
            
            names = cell(1,length(directed_adjacency_not_at_intersection));
            for i =1:nveh_not_at_intersection
                names{i} = strcat('veh',num2str(i));
            end

                
            Graph = digraph(directed_adjacency_not_at_intersection,names);
            
            
            priority_index  = toposort(Graph);
            priority_not_intersection = veh_not_at_intersection(priority_index); % ordered vehicle index w.r.t. priority
            
            priority(priority_not_intersection) = prio:nVeh;
            
            disp(['priority: ',num2str(priority)])
            [~,veh_index] = sort(priority);
            for group_idx = 1:nVeh
                groups(group_idx).members = veh_index(group_idx);
                if group_idx == 1
                    groups(group_idx).predecessors = [];
                else
                    groups(group_idx).predecessors = [groups(group_idx-1).predecessors groups(group_idx-1).members];
                end
            end
              
        end

    end
  
end














