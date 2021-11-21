classdef  random_priority < interface_priority
% random_priority  Instance of experiment interface used for simulation in matlab.
    
    properties (Access=private)

        
    end
    
    methods 
        function obj = random_priority(scenario)
            obj.scenario = scenario;
        end
        
        function groups = priority(obj)
            
%             groups = PB_random_groups(obj.scenario);
            groups = struct;
            nVeh = length(obj.scenario.vehicles);
            randomPrio = randperm(nVeh,nVeh); % random priority order 
%             randomPrio = 1:nVeh;
%             randomPrio = flip(1:nVeh); % random priority order

            for group_idx = 1:nVeh
                groups(group_idx).members = randomPrio(group_idx);
                if group_idx == 1
                    groups(group_idx).predecessors = [];
                else
                    groups(group_idx).predecessors = [groups(group_idx-1).predecessors groups(group_idx-1).members];
                end
            end

            
        end
      
        
    end
    


end