classdef random_priority < interface_priority
% random_priority  Instance of interface_priority used for dynamic priority
% assignment, randomly assign priority to vehicles
    
    properties (Access=private)
    end
    
    methods 
        function obj = random_priority() 
            obj.is_assign_unique_priority = false; % whether to asign unique priority
        end
        
        function [groups, directed_adjacency, priority_list] = priority(obj,scenario)

            directed_adjacency = scenario.adjacency(:,:,end);
            nVeh = scenario.options.amount;
            RandPrio = randperm(scenario.random_stream,nVeh,nVeh);

            for iVeh = 1:nVeh
                for jVeh = 1:nVeh
                    if directed_adjacency(iVeh,jVeh) && (RandPrio(iVeh) > RandPrio(jVeh))
                        directed_adjacency(iVeh,jVeh) = 0;
                    end
                end
            end

            [isDAG, Level] = kahn(directed_adjacency);

            assert( isDAG, 'Coupling matrix is not a DAG' );

            groups = PB_predecessor_groups(Level);

            % Assign prrority according to computation level
            % Vehicles with higher priorities plan trajectory before vehicles
            % with lower priorities            
            priority_list = obj.get_priority(groups);
        end
      
    end
    


end