classdef FCA_priority < interface_priority
% FCA  Instance of interface_priority used for dynamic priority
% assignment. Vehicles with more potential collisions on their future
% reference trajectory are assigned higher priorities.

    properties (Access=private)
        iter
    end
    
    methods 
        
        function obj = FCA_priority() 
        end
        
        function [veh_at_intersection,groups,directed_adjacency,priority_list] = priority(obj,scenario,iter)

            nVeh = length(scenario.vehicles);
            Hp = size(iter.referenceTrajectoryPoints,2);
            veh_at_intersection = [];

            %% assign priorities to vehicles based on future collision assessment
            
            % adjacency matrix
            adjacency= iter.adjacency;
            collisions = zeros(1,nVeh);
            
            veh = Vehicle();
            x_locals = [-1, -1,  1,  1] * (veh.Length/2 + scenario.options.offset);
            y_locals = [-1,  1,  1, -1] * (veh.Width/2 + scenario.options.offset);
            
            for nveh = 1: nVeh-1
                % position of nveh 
                nveh_x = iter.referenceTrajectoryPoints(nveh,:,1);
                nveh_y = iter.referenceTrajectoryPoints(nveh,:,2);
                refPath_n = [nveh_x;nveh_y]';
                nveh_yaw = calculate_yaw(refPath_n);
                % check adjacent vehicles 
                veh_adjacent = find(adjacency(nveh,:));
                
                %check the vehicles whose index is larger than the current vehicle, no repeated check
                veh_adjacent = veh_adjacent(veh_adjacent > nveh);
                                    
                for istep = 1:Hp 
                    % shape of nveh
                    [x_globals_n,y_globals_n] = translate_global(nveh_yaw(istep), nveh_x(istep), nveh_y(istep), x_locals, y_locals);
                    shape_n = [x_globals_n;y_globals_n];
                   
                    % check collistion between vehicles and static obstacles
                    if ~isempty(scenario.obstacles)
                        for i = 1:numel(scenario.obstacles)
                            if intersect_sat(shape_n,scenario.obstacles{i}) 
                                collisions(nveh) = collisions(nveh) + 1;
                            end
                        end
                    end
                    
                    % check collistion between vehicles and dynamic obstacles
                    if ~isempty(iter.dynamic_obstacle_area)
                        for i = 1:size(iter.dynamic_obstacle_area,1)
                            if intersect_sat(shape_n,iter.dynamic_obstacle_area{i,istep}) 
                                collisions(nveh) = collisions(nveh) + 1;
                            end
                        end
                    end
                    
                    % check collistion between two vehicles
                    for iveh = veh_adjacent              
                        % position of iveh
                        iveh_x = iter.referenceTrajectoryPoints(iveh,:,1);
                        iveh_y = iter.referenceTrajectoryPoints(iveh,:,2);
                        refPath_i = [iveh_x;iveh_y]';
                        iveh_yaw = calculate_yaw(refPath_i);
                    
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
            
            [~,FCAPrio] = sort(collisions,'descend'); % ordered vehicle index w.r.t. priority
            %disp(['collisions: ',num2str(collisions)])
            %disp(['priority_index: ',num2str(FCAPrio)])

            directed_adjacency = adjacency;

            for iVeh = 1:nVeh
                for jVeh = 1:nVeh
                    if directed_adjacency(iVeh,jVeh) && (FCAPrio(iVeh) > FCAPrio(jVeh))
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