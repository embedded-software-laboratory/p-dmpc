classdef (Abstract) interface_priority < handle
% interface_priority    Abstract class used for defining properties and methods used by priority based distributed controller.
    
    properties
        is_assign_unique_priority % whether to assign unique priority
    end
    

    methods (Abstract) 
        priority(obj)
    end
    methods
        function priority_list = get_priority(obj,CL_based_hierarchy)
            % Assign priorities to vehicles based on their computation levels
            % 
            % INPUT:
            %   CL_based_hierarchy: a struct contains in which computation
            %   level each vehicle is
            %
            %   is_assign_unique_priority: logical, whether assign unique
            %   priorities
            %
            % OUTPUT:
            %   priority_list: values of vehicles' priorities, smaller
            %   values indicate higher priorities

            nVeh = length([CL_based_hierarchy.members]);
            priority_list = zeros(1,nVeh);
            
            if obj.is_assign_unique_priority
                % each vehicle has unique priority
                prio = 1;
                for level_i = 1:length(CL_based_hierarchy)
                    vehs_in_level_i = CL_based_hierarchy(level_i).members; % vehicles in the selected computation level
                    for veh_i = vehs_in_level_i
                        priority_list(veh_i) = prio;
                        prio = prio + 1;
                    end
                end
            else
                % vehicles in the same computation level have the same priority
                for level_i = 1:length(CL_based_hierarchy)
                    vehs_in_level_i = CL_based_hierarchy(level_i).members;
                    priority_list(vehs_in_level_i) = level_i;
                end
            end
        end

    end
    
    
    
end
