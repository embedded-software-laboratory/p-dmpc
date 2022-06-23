classdef  random_priority < interface_priority
% random_priority  Instance of interface_priority used for dynamic priority
% assignment, randomly assign priority to vehicles
    
    properties (Access=private)

        
    end
    
    methods 
        function obj = random_priority() 
        end
        
        function [groups, directed_adjacency, priority_list] = priority(obj,scenario)
            
%             groups = PB_random_groups(scenario);
            groups = struct;
            nVeh = length(scenario.vehicles);
            randomPrio = randperm(nVeh,nVeh); % random priority order 

            directed_adjacency = zeros(nVeh,nVeh);
            
            for group_idx = 1:nVeh
                groups(group_idx).members = randomPrio(group_idx);
                
                if group_idx < nVeh
                    directed_adjacency(randomPrio(group_idx),randomPrio(group_idx+1)) = 1;
                end

                if group_idx == 1
                    groups(group_idx).predecessors = [];
                else
                    groups(group_idx).predecessors = [groups(group_idx-1).predecessors groups(group_idx-1).members];
                end
            end

            % Assign prrority according to computation level
            % Vehicles with higher priorities plan trajectory before vehicles
            % with lower priorities            
            priority_list = obj.get_priority(groups,obj.is_assign_unique_priority);
        end
      
        
    end
    


end