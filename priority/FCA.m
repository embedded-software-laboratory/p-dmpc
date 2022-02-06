classdef  FCA < interface_priority
% right_of_way  Instance of interface_priority used for dynamic priority
% assignment. For vehicles driving consectively, vehicles in the front are
% aasigned with higher priorities; For vehicles at the intersection, vehicles
% closer to the center of intersection are assigned with higher priorities. Vehicles at intersection keep higher priority and do not change their relative priority until they leave intersection
    
    properties (Access=private)
        iter
    end
    
    methods 
        
        function obj = FCA(scenario,iter)
            obj.scenario = scenario;
            obj.iter = iter;
        end
        
        function [veh_at_intersection,groups] = priority(obj)

            groups = struct;
            nVeh = length(obj.scenario.vehicles);
            Hp = size(obj.iter.referenceTrajectoryPoints,2);
            veh_at_intersection = [];

            %% assign priorities to vehicles based on future collision assessment
            
            % adjacency matrix
            adjacency= obj.scenario.semi_adjacency(:,:,end); 
            collisions = zeros(1,nVeh);
            
            veh = Vehicle();
            x_locals = [-1, -1,  1,  1] * (veh.Length/2 + obj.scenario.offset_length);
            y_locals = [-1,  1,  1, -1] * (veh.Width/2 + obj.scenario.offset_width);
            
            for nveh = 1: nVeh-1
                % position of nveh 
                nveh_x = obj.iter.referenceTrajectoryPoints(nveh,:,1);
                nveh_y = obj.iter.referenceTrajectoryPoints(nveh,:,2);
                refPath_n = [nveh_x;nveh_y]';
                nveh_yaw = calculate_yaw(refPath_n);
                % check adjacent vehicles 
                veh_adjacent = find(adjacency(nveh,:));
                
                %check the vehicles whose index is larger than the current vehicle, no repeated check
                veh_adjacent = veh_adjacent(veh_adjacent > nveh);
                
                for iveh = veh_adjacent              
                    % position of iveh
                    iveh_x = obj.iter.referenceTrajectoryPoints(iveh,:,1);
                    iveh_y = obj.iter.referenceTrajectoryPoints(iveh,:,2);
                    refPath_i = [iveh_x;iveh_y]';
                    iveh_yaw = calculate_yaw(refPath_i);
                    
                    for istep = 1:Hp 
                        % shape of nveh
                        [x_globals_n,y_globals_n] = translate_global(nveh_yaw(istep), nveh_x(istep), nveh_y(istep), x_locals, y_locals);
                        shape_n = [x_globals_n;y_globals_n];
                        
                        % shape of iveh
                        [x_globals_i,y_globals_i] = translate_global(iveh_yaw(istep), iveh_x(istep), iveh_y(istep), x_locals, y_locals);
                        shape_i = [x_globals_i;y_globals_i];
                        
                        % check if there is collision between nveh and iveh
                        if intersect_sat(shape_n,shape_i) 
                            collisions(nveh) = collisions(nveh) + 1;
                            collisions(iveh) = collisions(iveh) + 1;
                        end
                    end 
                end
            end        
            [~,priority_index] = sort(collisions,'descend'); % ordered vehicle index w.r.t. priority
            disp(['priority_index: ',num2str(priority_index)])
            
            [~,priority] = sort(priority_index); % ordered vehicle index w.r.t. priority
%             disp(['priority: ',num2str(priority)])
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