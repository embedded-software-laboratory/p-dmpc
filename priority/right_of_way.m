classdef  right_of_way < interface_priority
% random_priority  Instance of experiment interface used for simulation in matlab.
    
    properties (Access=private)
        isDAG
        topo_groups
        iter
        
    end
    
    methods 
        function obj = right_of_way(scenario,iter)
            obj.scenario = scenario;
            obj.iter = iter;
            
            if obj.scenario.assignPrios || isempty(obj.scenario.directed_coupling)
                [obj.isDAG, obj.topo_groups] = topological_sorting_coloring(obj.scenario.adjacency);
            else
                [obj.isDAG, obj.topo_groups] = kahn(obj.scenario.directed_coupling);
            end
            
        end
        
        function groups = priority(obj)

            groups = struct;
            nVeh = length(obj.scenario.vehicles);
            Hp = size(obj.iter.referenceTrajectoryPoints,2);
            intersection_center = [2.25, 2];
            prio = 1;
            priority = ones(1,nVeh); 
            lanelets_idx = zeros(nVeh, Hp);
            Veh = 1:nVeh;
            

            
            % identify vehicles at intersection
            for veh_idx = 1:nVeh   
                
                lanelets_idx(veh_idx,:) = lanelets_index(veh_idx,obj.iter,obj.scenario);
                is_at_intersection = (sum(ismember(lanelets_idx(veh_idx,:), obj.scenario.intersection_lanelets)) > 0);     
                if is_at_intersection
                    veh_at_intersection = [veh_at_intersection,veh_idx];
                    disp('there are vehicles at intersection')
                end

            end
            
            % check the distance to the center of the intersection and
            % assign higher priority to vehicles with smaller distance
            if is_at_intersection
                n_veh_at_intersection = length(veh_at_intersection);
            
                first_referenceTrajectoryPoints = [obj.iter.referenceTrajectoryPoints(veh,1,1),obj.iter.referenceTrajectoryPoints(veh,1,2)];
                diff = first_referenceTrajectoryPoints-intersection_center;
                distance_to_center = sqrt(diff(:,1).^2 + diff(:,2).^2);
                [~, index] = sort(distance_to_center,ascend);
                veh_at_intersection = veh_at_intersection(index);
                priority(veh_at_intersection)= 1:n_veh_at_intersection;

                % assign priority to vehicles that are not at intersection
                prio = n_veh_at_intersection + 1;
                veh_not_at_intersection = setdiff(Veh,veh_at_intersection);
            else
                veh_at_intersection = [];
                veh_not_at_intersection = setdiff(Veh,veh_at_intersection);
                
            end
            
            ordered_adjacent_group = [];
            veh_iterated = [];
            
            for veh_i = veh_not_at_intersection
                               
                ordered_group = adjacent_group_ordering(obj.scenario, obj.iter, veh_i, veh_at_intersection);
                ordered_adjacent_group = [ordered_adjacent_group,ordered_group]; 
                

                
                % if the first and the last one are not iterated yet,
                % further check these two first
                veh_first = ordered_adjacent_group(1);
                veh_end = ordered_adjacent_group(end);
                
                while ~ismember(veh_end, veh_iterated) || ~ismember(veh_first, veh_iterated)
                    if ~ismember(veh_end, veh_iterated)      
                        ordered_group = adjacent_group_ordering(obj.scenario, obj.iter, veh_end, veh_at_intersection);
                        ordered_adjacent_group = [ordered_adjacent_group,ordered_group];                   
                        veh_iterated = [veh_iterated, veh_end];

                    else

                        ordered_group = adjacent_group_ordering(obj.scenario, obj.iter, veh_first, veh_at_intersection);
                        ordered_adjacent_group = [ordered_group,ordered_adjacent_group];             
                        veh_iterated = [veh_iterated, veh_first];

                    end
                    
                    % remove the repeat elements and keep original orders
                    [~, index, ~] = unique(ordered_adjacent_group);
                    ordered_adjacent_group = ordered_adjacent_group(sort(index));
                    veh_first = ordered_adjacent_group(1);
                    veh_end = ordered_adjacent_group(end);
                    
                end 
                
                veh_iterated = [veh_iterated, veh_i];
                
        
                % if all vehicles are ordered, then stop iteration in advance
                if length(ordered_adjacent_group) == length(veh_not_at_intersection)
                    break
                end
                      
            end
            
            priority(ordered_adjacent_group) = prio:nVeh;
            
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














