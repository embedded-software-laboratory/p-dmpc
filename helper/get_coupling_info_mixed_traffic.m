function [coupling_weights, coupling_info] = get_coupling_info_mixed_traffic(scenario, iter)
% GET_COUPLING_INFO This function estimate the coupling information of
% vehicles for the mixed traffic scenario. Two vehicles are coupled if their reachable sets in the defined 
% prediction horizon (Hp) overlap. To reduce the checking effort, only the
% reachable sets in the last time step will be checked, which is almost
% sufficient. Heuristic method will be used to estimate the coupling degree
% between vehicles. More concrete,, the so-called "the shortest time to
% achieve a collision (STAC)" will be calculated, while a lower STAC means
% a higher coupling degree. 

    nVeh = scenario.nVeh;

    coupling_weights = zeros(nVeh,nVeh); % coupling weights of all coupling vehicle pair; higher value indicates stronger coupling
    
    % initialize a struct array to store information about coupling information
    coupling_info = struct('STAC_adapted',[],'weight',[]); 
    count = 1;

    state_indices = indices();
    
    if scenario.mixedTrafficCollisionAvoidanceMode == 2
        % autonomous vehicles check if their reachable sets intersect with reachable set of manual vehicles
        for iVeh = 1:nVeh
            if scenario.vehicle_ids(iVeh) ~= scenario.manual_vehicle_id && scenario.vehicle_ids(iVeh) ~= scenario.second_manual_vehicle_id
                % get the selected vehicle's position, speed, current trim and predicted lanelets 
                speed_i = iter.x0(iVeh, state_indices.speed);
                position_i = [iter.x0(iVeh,state_indices.x), iter.x0(iVeh,state_indices.y)];
                trim_i = iter.trim_indices(iVeh);
        
                for jVeh = 1:nVeh
                    if scenario.vehicle_ids(jVeh) == scenario.manual_vehicle_id || scenario.vehicle_ids(jVeh) == scenario.second_manual_vehicle_id
                        speed_j = iter.x0(jVeh, state_indices.speed);
                        position_j = [iter.x0(jVeh,state_indices.x), iter.x0(jVeh,state_indices.y)];
                        trim_j = iter.trim_indices(jVeh);
            
                        % if two the reachable set of an autonomous vehicle overlaps with the reachable set of a manual vehicle, they are considered
                        % as coupled. To save computation time, only the reachable sets at the last prediction horizon will be checked.
                        overlap_reachable_sets = intersect(iter.reachable_sets{iVeh,end}, iter.reachable_sets{jVeh,end});
                        area_overlap = area(overlap_reachable_sets);
                        if area_overlap > 1e-3 % a small threshold to tolerate measurement error of lenelet boundary                
                            % find the collision point in the overlapping area that minimizes the
                            % maximum distances to the position of the two vehicles 
                            center_between_two_points = (position_i+position_j)./2;
                            overlap_area_array = [overlap_reachable_sets.Vertices(:,1), overlap_reachable_sets.Vertices(:,2)];
            
                            is_in_overlap_area = inpolygon(center_between_two_points(1),center_between_two_points(2),...
                                overlap_area_array(:,1),overlap_area_array(:,2));
                            if is_in_overlap_area
                                % if the center point of two vehicles is inside the overlapping area, the target point is the center point
                                collision_point = center_between_two_points;
                            else
                                % otherwise compare the maximum times of all points on the sides of the overlapping area 
            
                                % without sqrt() to save coputation time
                                squared_distances_to_veh_i = sum((overlap_area_array-position_i).^2,2); 
                                squared_distances_to_veh_j = sum((overlap_area_array-position_j).^2,2);
                                % only interested in the maximum distance
                                max_distance_to_veh = max([squared_distances_to_veh_i,squared_distances_to_veh_j],[],2);
                                % find the minimum among the maximum distances
                                [~,find_target_point] = min(max_distance_to_veh);
                                collision_point = overlap_area_array(find_target_point,:);
                            end
            
                            distance_to_collision_i = norm(collision_point-position_i);
                            distance_to_collision_j = norm(collision_point-position_j);
            
                            time_to_collisionPoint_i = get_the_shortest_time_to_arrive(scenario.mpa, trim_i, distance_to_collision_i, scenario.dt);

                            if ((scenario.vehicle_ids(jVeh) == scenario.manual_vehicle_id) && scenario.manual_mpa_initialized) ...
                                || ((scenario.vehicle_ids(jVeh) == scenario.second_manual_vehicle_id) && scenario.second_manual_mpa_initialized)
                                mpa = scenario.vehicles(iVeh).vehicle_mpa;
                            else
                                mpa = scenario.mpa;
                            end

                            time_to_collisionPoint_j = get_the_shortest_time_to_arrive(mpa, trim_j, distance_to_collision_j, scenario.dt);
            
                            waiting_time = abs(time_to_collisionPoint_i-time_to_collisionPoint_j);
                            % actual STAC
                            STAC_actual = max(time_to_collisionPoint_i,time_to_collisionPoint_j);
                            % adapted STAC
                            STAC_adapted = STAC_actual + waiting_time;  
            
                            % store coupling information
                            coupling_info(count).STAC_adapted = STAC_adapted;
                            coupling_info(count).weight = weighting_function(STAC_adapted);
            
                            if time_to_collisionPoint_i >= time_to_collisionPoint_j
                                coupling_weights(iVeh,jVeh) = coupling_info(count).weight; 
                            else 
                                % manual vehicle would be responsible for collision
                                % include RSS-based rules here
                                % if autonomous vehicle has enough time for emergency braking, then it has to perform it
                                % otherwise break coupling and end scenario for all vehicles (also manual)
                                %coupling_weights(veh_j,veh_i) = coupling_info(count).weight; 
                                
                                %if consecutive lanes or same lanes

                                %else
                                    % distance for autonomous vehicle to perform emergency breaking maneuver
                                    emergency_braking_distance = get_emergency_braking_distance(scenario.mpa, trim_i, scenario.dt);

                                    % time needed for autonomous vehicle to perform emergency braking maneuver
                                    emergency_braking_time = get_emergency_braking_time(scenario.mpa, trim_i, emergency_braking_distance, scenario.dt);

                                    % time of manual vehicle to travel emergency braking distance
                                    %manual_vehicle_time_during_braking = get_the_shortest_time_to_arrive(mpa, trim_j, emergency_braking_distance, scenario.dt);

                                    % distance travelled by manual vehicle during autonomous emergency brake
                                    manual_vehicle_distance_during_braking = speed_j * emergency_braking_time;

                                    % distance of autonomous vehicle to collision point after braking
                                    distance_autonomous = distance_to_collision_i - emergency_braking_distance;

                                    % distance of manual vehicle to collision point after autonomous vehicle braking
                                    distance_manual = distance_to_collision_j - manual_vehicle_distance_during_braking;

                                    if distance_autonomous > 0
                                        if distance_autonomous < distance_manual 
                                            % no emergency braking necessary
                                        elseif distance_autonomous > (distance_manual + 0.2)
                                            % if distance autonomous after braking larger that distance of manual vehicle after braking to collision point plus safety
                                            % threshold (vehicle length), then autonomous vehicle should brake
                                            coupling_weights(iVeh,jVeh) = coupling_info(count).weight; 
                                        else
                                            % collision unavoidable, stop all vehicles immediately
                                        end
                                    end
                                %end        
                            end
                            count = count + 1; 
            
                        end   
                    end
                end
            end
    
        end
    else
        % autonomous vehicles check if their reachable sets intersect with reachable set of all vehicles
    end

end
    
    
    %% local function
    function coupling_weight = weighting_function(STAC,sensitive_factor)
        % returns the coupling weight based on the shortest time to achieve a
        % collision (STAC)
        if nargin==1
            sensitive_factor = 1; % sensitive factor used in calculating the coupling weights. The bigger, the more sensitive to the STAC. Default value 1.
        end
        coupling_weight = exp(-sensitive_factor*STAC); % negative exponential function
    end
    
    
    %% local function
    function is_leader = determine_leader_and_follower(veh_i, veh_j, time_enter_intersection, directed_adjacency_old, time_to_collisionPoint_i, time_to_collisionPoint_j)
    % Determines who is the leader and who is the follower for vehicles at the intersection. 
    % Vehicle enters the intersection earlier has a higher priority. If both
    % vheicle enter the intersection at the same time, their previous
    % coupling direction are kept. If they are not coupled at previous time
    % step, vehicle who can arrives the point of intersection has a higher
    % priority. 
    
        if time_enter_intersection(veh_i) < time_enter_intersection(veh_j)
            is_leader = true; 
        elseif time_enter_intersection(veh_i) > time_enter_intersection(veh_j)
            is_leader = false; 
        else
            % if enter the intersection at the same time, keep the coupling direction as before
            if directed_adjacency_old(veh_i,veh_j) == 1
                is_leader = true; 
            elseif directed_adjacency_old(veh_j,veh_i) == 1
                is_leader = false;
            else
                % if they are not coupled at previous time step, vehicle who can arrives the collision point earlier has a higher priority
                if time_to_collisionPoint_i <= time_to_collisionPoint_j
                    is_leader = true; 
                else
                    is_leader = false; 
                end
            end
        end
    
    end
    
    