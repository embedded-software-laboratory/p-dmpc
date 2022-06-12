classdef  mixed_traffic_priority < interface_priority
% mixed_traffic_priority  Instance of interface_priority used for priority
% assignment, fixed priority according to vehicle ids
% first manual vehicle gets highest priority, second manual vehicle second highest priority
    
    properties (Access=private)

        
    end
    
    methods 
        function obj = mixed_traffic_priority(scenario)
            obj.scenario = scenario;
        end
        
        function groups = priority(obj)
            groups = struct;
            nVeh = length(obj.scenario.vehicles);

            prios = zeros(1, nVeh);
            for i = 1:nVeh
                if obj.scenario.vehicle_ids(i) == obj.scenario.manual_vehicle_id
                    prios(end) = i;
                elseif obj.scenario.vehicle_ids(i) == obj.scenario.second_manual_vehicle_id
                    prios(end-1) = i;
                else
                    for j = 1:nVeh
                        if prios(j) == 0
                            prios(j) = i;
                            break
                        end
                    end
                end
            end

            for group_idx = 1:nVeh
                groups(group_idx).members = prios(group_idx);
                if group_idx == 1
                    groups(group_idx).predecessors = [];
                else
                    groups(group_idx).predecessors = [groups(group_idx-1).predecessors groups(group_idx-1).members];
                end
            end

            
        end
      
        
    end
    


end