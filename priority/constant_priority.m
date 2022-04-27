classdef  constant_priority < interface_priority
% constant_priority  Instance of interface_priority used for priority
% assignment, fixed priority according to vehicle ids
    
    properties (Access=private)

        
    end
    
    methods 
        function obj = constant_priority(scenario)
            obj.scenario = scenario;
        end
        
        function groups = priority(obj)
            
%             groups = PB_random_groups(obj.scenario);
            groups = struct;
            nVeh = length(obj.scenario.vehicles);
            ConstPrio = 1:nVeh; % random priority order 

            for group_idx = 1:nVeh
                groups(group_idx).members = ConstPrio(group_idx);
                if group_idx == 1
                    groups(group_idx).predecessors = [];
                else
                    groups(group_idx).predecessors = [groups(group_idx-1).predecessors groups(group_idx-1).members];
                end
            end

            
        end
      
        
    end
    


end