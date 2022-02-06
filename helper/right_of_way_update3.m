classdef  right_of_way_update3 < interface_priority
% right_of_way  Instance of interface_priority used for dynamic priority
% assignment. For vehicles driving consectively, vehicles in the front are
% aasigned with higher priorities; For vehicles at the intersection, vehicles
% closer to the center of intersection are assigned with higher priorities. Vehicles at intersection keep higher priority and do not change their relative priority until they leave intersection
    
    properties (Access=private)
        iter
    end
    
    methods 
        
        function obj = right_of_way_update3(scenario,iter)
            obj.scenario = scenario;
            obj.iter = iter;
        end
        
        function [veh_at_intersection,groups] = priority(obj,last_veh_at_intersection)

            groups = struct;
            nVeh = length(obj.scenario.vehicles);
            Hp = size(obj.iter.referenceTrajectoryPoints,2);
            intersection_center = [2.25, 2];
            lanelets_idx = zeros(nVeh, Hp);
            veh_at_intersection = [];
            directed_adjacency = zeros(nVeh,nVeh);
            %% assign priorities to vehicles based on driving order
            
            % semi_adjacency matrix
            semi_adjacency= obj.scenario.semi_adjacency(:,:,end); 
%             disp('semi_adjacency')
%             disp(semi_adjacency)
            % update the undirected adjacency matrix to a directed adajacency matrix
            
            for nveh = 1: nVeh-1

                % check semi_adjacent vehicles (i.e. vehicles driving consecutively)
                veh_semi_adjacent = find(semi_adjacency(nveh,:));
                
                %check the vehicles whose index is larger than the current vehicle, no repeated check
                veh_semi_adjacent = veh_semi_adjacent(veh_semi_adjacent > nveh);
                
                for iveh = veh_semi_adjacent  
                    is_leading_vehicle = check_driving_order(obj.iter, nveh, iveh);
                    if is_leading_vehicle
                        directed_adjacency(nveh, iveh) = 1;
                    else
                        directed_adjacency(iveh, nveh) = 1;            
                    end                  
                end         
            end
            
            %% identify vehicles at intersection and assign priorities accordingly
            for veh_idx = 1:nVeh   
                lanelets_idx(veh_idx,:) = lanelets_index(veh_idx,obj.iter,obj.scenario);
                is_at_intersection = (sum(ismember(lanelets_idx(veh_idx,:), obj.scenario.intersection_lanelets)) > 0);
                if is_at_intersection
                    veh_at_intersection = [veh_at_intersection,veh_idx];
                    n_veh_at_intersection = length(veh_at_intersection);
                end
 
            end
            
%             disp(['veh_at_intersection: ', num2str(veh_at_intersection)])
            
            adjacency= obj.scenario.adjacency(:,:,end);
            
            % assign priorities to vehicles at the intersection, but not driving consecutively
            if ~isempty(veh_at_intersection)
                
%                 disp(['last_veh_at_intersection: ', num2str(last_veh_at_intersection)])
                % if there were vehicles at intersection at last step, remained vehicles at intersection keep priorities as last step,
                % and assign lower priorities to new vehicles coming into intersection based on the distance to the intersection center

                [~, idx,~] = intersect(last_veh_at_intersection,veh_at_intersection);
                remained_veh_at_intersection = last_veh_at_intersection(sort(idx));

                % new incoming vehicles at intersection
                new_veh_at_intersection = setdiff(veh_at_intersection, remained_veh_at_intersection);

                % assign priorities to new vehicles at intersection based on their distance to the intersection center
                if ~isempty(new_veh_at_intersection)

                    first_referenceTrajectoryPoints = [obj.iter.referenceTrajectoryPoints(new_veh_at_intersection,1,1),obj.iter.referenceTrajectoryPoints(new_veh_at_intersection,1,2)];
                    diff = first_referenceTrajectoryPoints - intersection_center;
                    distance_to_center = sqrt(diff(:,1).^2 + diff(:,2).^2);
                    [~, index] = sort(distance_to_center,'ascend');

                    % sort the vehicles at intersection based on their distance to the intersection center
                    % and assign higher priority to the vehicle closer to the intersection center
                    new_veh_at_intersection = new_veh_at_intersection(index);
                    veh_at_intersection = [remained_veh_at_intersection,new_veh_at_intersection];
                else                      
                    veh_at_intersection = remained_veh_at_intersection;
                end
       
                if n_veh_at_intersection > 1
                    
                    for veh_idx = 1:n_veh_at_intersection - 1 
                        veh_adjacent = find(adjacency(veh_at_intersection(veh_idx),:));
                        veh_semi_adjacent = find(semi_adjacency(veh_at_intersection(veh_idx),:));

                        % find the vehicles that do not drive consecutively
                        veh_adjacent_diff = setdiff(veh_adjacent, veh_semi_adjacent);

                        for veh_adj_idx = (veh_idx+1):n_veh_at_intersection
                            if ismember(veh_at_intersection(veh_adj_idx),veh_adjacent_diff)
                                directed_adjacency(veh_at_intersection(veh_idx),veh_at_intersection(veh_adj_idx)) = 1;
                                directed_adjacency(veh_at_intersection(veh_adj_idx),veh_at_intersection(veh_idx)) = 0;
                            end 
                        end       
                    end
                end
   
            end
               
            names = cell(1,length(directed_adjacency));
            for i =1:nVeh
                names{i} = strcat('veh',num2str(i));
            end
 
            Graph = digraph(directed_adjacency);
%             disp(directed_adjacency)
            
            %check if there is any cycles in the directed graph
            cycles = allcycles(Graph);
            
            
            %% make sure the most number of vehicles in the first of all cycles
            
            while ~isempty(cycles)
                % break the cycle between vehicles which have the largest distance 
%                 disp('there are cycles in the directed graph')
                cyclic_vehicles = cycles{1};
                n_cyclic_vehicles = length(cyclic_vehicles);  
                distance = 0;
                
                for vehi = 1:n_cyclic_vehicles
                    if vehi < n_cyclic_vehicles
                        distance_i = check_distance(obj.iter, cyclic_vehicles(vehi),cyclic_vehicles(vehi+1));
                    else
                        distance_i = check_distance(obj.iter, cyclic_vehicles(vehi),cyclic_vehicles(1));
                    end
                    
                    if distance_i > distance
                        distance = distance_i;
                        veh_to_break = vehi;
                    end
                    
                end
                
                if veh_to_break < n_cyclic_vehicles
                    i = cyclic_vehicles(veh_to_break);
                    j = cyclic_vehicles(veh_to_break + 1);
                else
                    i = cyclic_vehicles(veh_to_break);
                    j = cyclic_vehicles(1);
                    
                end

                directed_adjacency(i,j) = 0;
%                 directed_adjacency(j,i) = 0;
                Graph = digraph(directed_adjacency);
                cycles = allcycles(Graph);
            end
            
            
%             while ~isempty(cycles)
%                 
%                 n_cycles = length(cycles);% 不一定有用
%                 largest_distance = zeros(n_cycles,1);
%                 veh_to_break_in_cycle = zeros(n_cycles,1);
%                 n_cyclic_vehicles = zeros(n_cycles,1);  
%                 disp(['cycles: ',num2str(cycles{1})])
%                 % break the cycle between vehicles which have the largest distance 
%                 disp('there are cycles in the directed graph')
%                 for n = 1:n_cycles
%                     cyclic_vehicles = cycles{n};
%                     n_cyclic_vehicles(n,1) = length(cyclic_vehicles);  
%                     distance = 0;
%                     for vehi = 1:n_cyclic_vehicles(n,1)
%                         if vehi < n_cyclic_vehicles(n,1)
%                             distance_i = check_distance(obj.iter, cyclic_vehicles(vehi),cyclic_vehicles(vehi+1));
%                         else
%                             distance_i = check_distance(obj.iter, cyclic_vehicles(vehi),cyclic_vehicles(1));
%                         end
%                         if distance_i > distance
%                             distance = distance_i;
%                             veh_to_break_in_cycle(n,1) = cyclic_vehicles(vehi);
%                         end
%                     end
%                     largest_distance(n,1) = distance;
%                 end
%                 [sorted_distance, cycle_index] = sort(largest_distance,'descend');
%                 cycle_to_break = cycle_index(end);
%                 veh_to_break = veh_to_break_in_cycle(cycle_to_break);
%                 
%                 veh_to_break_index = find(cycles{cycle_to_break} == veh_to_break);
%                 
% %                 disp(['veh_to_break',num2str(veh_to_break)])  
%                 if veh_to_break_index < n_cyclic_vehicles(cycle_to_break)
%                     i = cycles{cycle_to_break}(veh_to_break_index);
%                     j = cycles{cycle_to_break}(veh_to_break_index + 1);
%                 else
%                     i = cycles{cycle_to_break}(veh_to_break_index);
%                     j = cycles{cycle_to_break}(1);    
%                 end
%                 directed_adjacency(i,j) = 0;
% %                 directed_adjacency(j,i) = 0;
%                 Graph = digraph(directed_adjacency);
%                 cycles = allcycles(Graph);
%             end
            
            priority_index  = toposort(Graph);% compare the function from helper/kahn.m
            disp(['prio_index: ',num2str(priority_index)])
            [~,priority] = sort(priority_index); % ordered vehicle index w.r.t. priority
           
            disp(['priority: ',num2str(priority)])
%             disp(['veh_index: ',num2str(priority_index)])
            for group_idx = 1:nVeh
                groups(group_idx).members = priority_index(group_idx);
                if group_idx == 1
                    groups(group_idx).predecessors = [];
                else
                    groups(group_idx).predecessors = [groups(group_idx-1).predecessors groups(group_idx-1).members];
                end
            end
              
        end

    end
  
end