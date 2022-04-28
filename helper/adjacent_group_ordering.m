function ordering = adjacent_group_ordering(scenario, iter, veh_i, veh_at_intersection)
%adjacent_group_ordering   returns the ordered adjacent group by checking
%their driving order. Vehicles in the front are assigned higher priorities.

        % adjacent vehicles (exclude vehicles at intersection, because all vehicles at intersection have higher priorities than other vheicles) 
        adjacent_vehicles = find(scenario.adjacency(veh_i,:,end));
        adjacent_vehicles = setdiff(adjacent_vehicles,veh_at_intersection);
        adjacent_group = [veh_i,adjacent_vehicles];
        

        for n = 0:length(adjacent_group)-1
            
            for i = 1:length(adjacent_group)-1

                first_refPoint_x = iter.referenceTrajectoryPoints(adjacent_group(i),1,1);
                first_refPoint_y = iter.referenceTrajectoryPoints(adjacent_group(i),1,2);
                first_refPoint = [first_refPoint_x,first_refPoint_y];

                second_refPoint_x = iter.referenceTrajectoryPoints(adjacent_group(i),end,1);
                second_refPoint_y = iter.referenceTrajectoryPoints(adjacent_group(i),end,2);
                second_refPoint = [second_refPoint_x,second_refPoint_y];

                driving_direction = second_refPoint - first_refPoint;

                % reference trajectory points of adjacent vehicles
                first_refPoint_adj_x = iter.referenceTrajectoryPoints(adjacent_group(i+1),1,1);
                first_refPoint_adj_y = iter.referenceTrajectoryPoints(adjacent_group(i+1),1,2);
                first_refPoint_adj = [first_refPoint_adj_x,first_refPoint_adj_y];

                vehicle_distance_direction = first_refPoint - first_refPoint_adj;

                % check if driving consecutively
                is_leading_vehicle = (dot(driving_direction, vehicle_distance_direction) > 0);

                if ~is_leading_vehicle
                    adjacent_group([i,i+1]) = adjacent_group([i+1,i]);
                end

            end
        end     
        ordering = adjacent_group;
end