classdef  right_of_way_priority < interface_priority
% right_of_way  Instance of interface_priority used for dynamic priority
% assignment. For vehicles driving consectively, vehicles in the front are
% aasigned higher priorities; For vehicles crossing the intersection, vehicles
% closer to the center of intersection are assigned with higher priorities. 
% Vehicles at intersection keep higher priority and do not change their relative priority until they leave intersection
    
    properties (Access=private)
        iter
    end
    
    methods 
        
        function obj = right_of_way_priority(scenario,iter)
            obj.scenario = scenario;
            obj.iter = iter;
        end
        
        function [veh_at_intersection,groups,edge_to_break] = priority(obj)

            groups = struct;
            nVeh = length(obj.scenario.vehicles);
            Hp = size(obj.iter.referenceTrajectoryPoints,2);
            intersection_center = [2.25, 2];
            lanelets_idx = zeros(nVeh, Hp);
            veh_at_intersection = [];
            directed_adjacency = zeros(nVeh,nVeh);
            %% assign priorities to vehicles driving consecutively
            
            % semi_adjacency matrix
            semi_adjacency= obj.scenario.semi_adjacency(:,:,end); 

            
            % update the undirected adjacency matrix to a directed adajacency matrix
            for nveh = 1: nVeh-1

                % check semi_adjacent vehicles (i.e. vehicles driving consecutively)
                veh_semi_adjacent = find(semi_adjacency(nveh,:));
                
                % check the vehicles whose index is larger than the current vehicle, no repeated check
                veh_semi_adjacent = veh_semi_adjacent(veh_semi_adjacent > nveh);
                
                for iveh = veh_semi_adjacent  
                    is_leading_vehicle = check_driving_order(obj.scenario,obj.iter, nveh, iveh);
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
            
            
            adjacency= obj.scenario.adjacency(:,:,end);
            
            % assign priorities to vehicles crossing the intersection
            % (i.e., at the intersection, but not driving consecutively)
            if ~isempty(veh_at_intersection)
                % if there were vehicles at intersection at last step, remained vehicles at intersection keep priorities as last step,
                % and assign lower priorities to new vehicles coming into intersection based on the distance to the intersection center

                [~, idx,~] = intersect(obj.scenario.last_veh_at_intersection,veh_at_intersection);
                remained_veh_at_intersection = obj.scenario.last_veh_at_intersection(sort(idx));

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
%                                 directed_adjacency(veh_at_intersection(veh_adj_idx),veh_at_intersection(veh_idx)) = 0;
                            end 
                        end       
                    end
                end
   
            end
               
            % Convert the Matrix "directed_adjacency" to a directed Graph
            Graph = digraph(directed_adjacency);
            
            %check if there is any cycles in the directed graph
            cycles = allcycles(Graph);
            
            
            %% Convert directed cyclic graph to directed acyclic graph
            edge_to_break = {};
            while ~isempty(cycles)
                % break the cycle between vehicles which have the largest distance 
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
                edge_to_break{end+1} = [i,j];

                directed_adjacency(i,j) = 0;
                % construct a new graph and check the cycles in the graph
                Graph = digraph(directed_adjacency);
                cycles = allcycles(Graph);
            end
            
           % calculate computation levels using kahn algorithm(topological ordering)
            [~, L] = kahn(directed_adjacency);
            computation_levels = size(L,1);
%                 % visualize the directed graph  
%                 figure(); plot(Graph) 


            % construct the priority groups
            for group_idx = 1:computation_levels
                groups(group_idx).members = find(L(group_idx,:));
                if group_idx == 1
                    groups(group_idx).predecessors = [];
                else
                    groups(group_idx).predecessors = [groups(group_idx-1).predecessors groups(group_idx-1).members];
                end
            end    
              
        end

    end
  
end