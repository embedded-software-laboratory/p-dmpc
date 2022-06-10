function [all_veh_at_intersection, coupling_weights, coupling_infos, time_enter_intersection] = get_coupling_infos_version2(scenario, iter)
% GET_COUPLING_INFOS This function estimate the coupling information of
% vehicles. Two vehicles are coupled if their reachable sets in the defined 
% prediction horizon (Hp) overlap. To reduce the checking effort, only the
% reachable sets in the last time step will be checked, which is almost
% sufficient. Heuristic method will be used to estimate the coupling degree
% between vehicles. More concrete,, the so-called "the shortest time to
% achieve a collision (STAC)" will be calculated, whilea a lower STAC means
% a higher coupling degree. 

    nVeh = scenario.nVeh;
    lanelets = scenario.lanelets;
    lanelet_relationships = scenario.lanelet_relationships;

    directed_adjacency_old = zeros(nVeh,nVeh); % previous directed adjacency matrix
    if scenario.k>1
        for i = 1:length([scenario.coupling_infos.idLeader])
            idLeader = scenario.coupling_infos(i).idLeader;
            idFollower = scenario.coupling_infos(i).idFollower;
            directed_adjacency_old(idLeader,idFollower) = 1;
        end
    end

    time_enter_intersection = scenario.time_enter_intersection;
    if isempty(time_enter_intersection)
        % this is empty only at the initial time step
        time_enter_intersection = inf*ones(1,nVeh); % time step when vehicle enters the intersection
    end

    last_veh_at_intersection = scenario.last_veh_at_intersection;

    % vehicles are considered as at the intersection if their distances to
    % the intersection center point is smaller than a certain value
    distances_to_center = sqrt(sum((iter.x0(:,1:2) - scenario.intersection_center).^2,2));
    all_veh_at_intersection = find(distances_to_center < 1.1);

    new_veh_at_intersection = setdiff(all_veh_at_intersection, last_veh_at_intersection);
    if ~isempty(new_veh_at_intersection)
        % disp('debug')
    end

    veh_leave_intersection = setdiff(last_veh_at_intersection,all_veh_at_intersection);
    time_enter_intersection(new_veh_at_intersection) = scenario.k;
    time_enter_intersection(veh_leave_intersection) = inf; % set to inf if vehicle leaves the intersection
    
    coupling_weights = zeros(nVeh,nVeh); % coupling weights of all coupling vehicle pair; higher value indicates stronger coupling
    
    % initialize a struct array to store information about coupling information
    coupling_infos = struct('idLeader',[],'idFollower',[],'type',[],'lanelet_relationship',[],'speedLeader',[],'speedFollower',[],'positionLeader',[],'positionFollower',[],'STAC_adapted',[]); 
    count = 1;

    state_indices = indices();
    
    for veh_i = 1:(nVeh-1)
        % get the selected vehicle's position, speed, current trim and predicted lanelets 
        speed_i = iter.x0(veh_i, state_indices.speed);
        position_i = [iter.x0(veh_i,state_indices.x), iter.x0(veh_i,state_indices.y)];
        trim_i = iter.trim_indices(veh_i);
        predicted_lanelets_i = iter.predicted_lanelets{veh_i};

        for veh_j = (veh_i+1):nVeh
            speed_j = iter.x0(veh_j, state_indices.speed);
            position_j = [iter.x0(veh_j,state_indices.x), iter.x0(veh_j,state_indices.y)];
            trim_j = iter.trim_indices(veh_j);
            predicted_lanelets_j = iter.predicted_lanelets{veh_j};

            overlap_reachable_sets = intersect(iter.reachable_sets{veh_i,end}, iter.reachable_sets{veh_j,end});
            area_overlap = area(overlap_reachable_sets);

            if scenario.k>=62 && veh_i == 8
                disp('')
            end
            if area_overlap > 1e-3 % a small threshold to tolerate measurement error of lenelet boundary
                stop_flag = false;
                % the selected two vehicles are considered as coupled if their reachable sets overlap
                for predicted_lanelet_i = predicted_lanelets_i                    
                    for predicted_lanelet_j = predicted_lanelets_j
                        % get the relationship of their predicted lanelets and the possible collision type 
                        if predicted_lanelet_i == predicted_lanelet_j
                            is_same_lanelet = true; % the same lanelet
                            % If the same lanelet -> rear-end collision
                            is_rear_end_collision = true;
                            is_side_impact_collision = false;
                            is_forking_lanelets = false;
                            lanelet_relationship.type = LaneletRelationshipType.type_6; % same lanelet
                        else
                            is_same_lanelet = false;
                            % find if there exists lanelet pair that has a certain relationship in the struct array `lanelet_relationships`
                            % NOTE that only adjacent lanelet pairs with certain relationships will be stored in `lanelet_relationships`
                            idx_lanelet_pair = find_idx_lanelet_pair(predicted_lanelet_i,predicted_lanelet_j,lanelet_relationships);
                            if ~isempty(idx_lanelet_pair)
                                is_find_lanelet_relationship = true;
                                lanelet_relationship = lanelet_relationships(idx_lanelet_pair);

                                % If forking lanelets (relationship type 4) or successive (relationship type 1) lanelets or 
                                % adjacent left/right lanelets (relationship type 2) -> rear-end collision
                                is_forking_lanelets = strcmp(lanelet_relationship.type, LaneletRelationshipType.type_4);
                                is_rear_end_collision = strcmp(lanelet_relationship.type, LaneletRelationshipType.type_1) ||...
                                    strcmp(lanelet_relationship.type, LaneletRelationshipType.type_2) || is_forking_lanelets;
                                
                                % if merging (relationship type 3) lanelets or intersecting lanelets (relationship type 5) -> side-impact collision
                                is_side_impact_collision = strcmp(lanelet_relationship.type, LaneletRelationshipType.type_3) ||...
                                    strcmp(lanelet_relationship.type, LaneletRelationshipType.type_5);
                            else
                                if predicted_lanelet_i==predicted_lanelets_i(end) && predicted_lanelet_j==predicted_lanelets_j(end)
                                    % If no lanelet relationship is found until the last predicted lanelet pair
                                    % 1. Set the center point of the overlapping area as collision point
                                    % 2. Set lanelet relationship to be intersecting
                                    % 3. Set collision type to be side-impact collision (normally lanelet relationship will always be found for rear-end collision)
%                                     disp(['No lanelet relationship can be found for coupled vehicle ' num2str(veh_i) ' and ' num2str(veh_j) '.' newline...
%                                         'Centroid of the overlapping area of their reachable sets will be used as collision point.'])
                                    is_find_lanelet_relationship = false;
                                    is_rear_end_collision = false;
                                    is_side_impact_collision = true;
                                    lanelet_relationship.type = LaneletRelationshipType.type_5; % intersecting lanelets
                                    [x_centroid,y_centroid] = centroid(overlap_reachable_sets);
                                    lanelet_relationship.point = [x_centroid,y_centroid];
                                else
                                    % jump to the next predicted lanelet
                                    continue
                                end
                            end
                        end

                        % check if both vehicles are at the intersection 
                        if ismember(veh_i,all_veh_at_intersection) && ismember(veh_j,all_veh_at_intersection)
                            is_both_at_intersection = true;
                        else
                            is_both_at_intersection = false;
                        end

                        % center line of the lanelet
                        lanelet_x_i = lanelets{predicted_lanelet_i}(:,LaneletInfo.cx); lanelet_y_i = lanelets{predicted_lanelet_i}(:,LaneletInfo.cy);
                        lanelet_x_j = lanelets{predicted_lanelet_j}(:,LaneletInfo.cx); lanelet_y_j = lanelets{predicted_lanelet_j}(:,LaneletInfo.cy);

                        if is_rear_end_collision
                            % If the same lanelet or successive (relationship type 1) lanelets or adjacent left/right lanelets (relationship type 2) -> rear-end collision
                            coupling_infos(count).type = CollisionType.type_1; % rear-end collision

                            if is_forking_lanelets
                                % In forking lanelets, rear-end collision is possible around the forking point
                                % First, assume they are driving in a straight line and calculate the STAC.
                                % Then estimate the points that the leader and the follower would stop at if it drives at its own lanelet.
                                % They are only considered as coupled if the distance between two stopping points are smaller than vehicle's length.
                                forking_point = lanelet_relationship.point;
                                
                                distance_to_starting_point_i = norm(position_i-forking_point,2);
                                distance_to_starting_point_j = norm(position_j-forking_point,2);

                                is_leader = determine_leader_and_follower(veh_i, veh_j, distance_to_starting_point_i, distance_to_starting_point_j, is_both_at_intersection, time_enter_intersection, directed_adjacency_old, is_forking_lanelets);

                                if is_leader                       
                                    trim_leader = trim_i;                       trim_follower = trim_j;
                                    position_leader = position_i;               position_follower = position_j;         
                                    lanelet_leader = [lanelet_x_i,lanelet_y_i]; lanelet_follower = [lanelet_x_j,lanelet_y_j];
                                else
                                    trim_leader = trim_j;                       trim_follower = trim_i;
                                    position_leader = position_j;               position_follower = position_i;                                    
                                    lanelet_leader = [lanelet_x_j,lanelet_y_j]; lanelet_follower = [lanelet_x_i,lanelet_y_i];
                                end

                                % current distance between two vehicles
                                distance_two_vehs = norm(position_i-position_j,2);
                                % calculate the shortest time to achieve a collision
                                [STAC, waiting_time, distance_traveled_leader, distance_traveled_follower] = get_the_shortest_time_to_catch(scenario.mpa, trim_leader, trim_follower, distance_two_vehs, scenario.dt);

                                stop_point_leader = get_stop_point_after_travel_certain_distance(position_leader,lanelet_leader,distance_traveled_leader);
                                stop_point_follower = get_stop_point_after_travel_certain_distance(position_follower,lanelet_follower,distance_traveled_follower);

                                distance_stop_points = norm(stop_point_leader-stop_point_follower);
                                if distance_stop_points > Vehicle().Length
                                    % the selected two vehicles will never collide with each other since their stopping points are far away
                                    continue
                                end
                                STAC_adapted = STAC + waiting_time;
                            else
                                % Otherwise vehicles are driving at the same lanelets or successive lanelets 
                                
                                % Get the endpoint of the lanelet
                                if is_same_lanelet
                                    endpoint = [lanelet_x_i(end), lanelet_y_i(end)];
                                else
                                    endpoint = lanelet_relationship.point;
                                end
    
                                distance_to_endpoint_i = norm(position_i-endpoint,2);
                                distance_to_endpoint_j = norm(position_j-endpoint,2);
                                
                                % determine who is the leader 
                                is_leader = determine_leader_and_follower(veh_i, veh_j, distance_to_endpoint_i, distance_to_endpoint_j, is_both_at_intersection, time_enter_intersection, directed_adjacency_old, is_forking_lanelets);
                                
                                if is_leader                        
                                    trim_leader = trim_i; trim_follower = trim_j;
                                else
                                    trim_leader = trim_j; trim_follower = trim_i;
                                end
    
                                % current distance between two vehicles
                                distance_two_vehs = norm(position_i-position_j,2);
                                % calculate the shortest time to achieve a collision
                                [STAC, waiting_time, ~, ~] = get_the_shortest_time_to_catch(scenario.mpa, trim_leader, trim_follower, distance_two_vehs, scenario.dt);
                                STAC_adapted = STAC + waiting_time;
                            end
                        elseif is_side_impact_collision
                            coupling_infos(count).type = CollisionType.type_2; % side-impact collision

                            % For side-impact collision, we calculate the adapted STAC, which is the sum of the actual STAC and a waiting time. 
                            % The calculation procedure is the following: Firstly, the actual STAC is the shortest time needed to achieve a 
                            % collision at the point of intersection. This is calculated by letting both vehicles take a full acceleration to 
                            % arrive at the point of intersection. The first arrived vehicle has to stop at the point of intersection and wait 
                            % for the second arrived vehicle, where the waiting time is the second part of the adapted STAC.
                            if is_find_lanelet_relationship
                                collision_point = lanelet_relationship.point;
                                % from a curve to calculate the arc diatance between vehicle's current position and the collision point, 
                                % which starts from the starting point of the lanelet and ends at the collision point 
                                if strcmp(lanelet_relationship.type, LaneletRelationshipType.type_3)
                                    % collision point of the merging lanelets is both lanelets' endpoint, thus the target curve is the whole lanelet
                                    curve_x_i = lanelet_x_i; curve_y_i = lanelet_y_i;
                                    curve_x_j = lanelet_x_j; curve_y_j = lanelet_y_j;
                                elseif strcmp(lanelet_relationship.type, LaneletRelationshipType.type_5)
                                    % Collision point of the intersecting lanelets is the crosspoint, which could be in the middle of the lanelets
                                    % First, find the two closest points to the crosspoint on the lanelet
                                    squared_distances_to_crosspoint_i = sum(([lanelet_x_i,lanelet_y_i]-collision_point).^2,2);
                                    [~,idx_closest_two_point_i] = mink(squared_distances_to_crosspoint_i,2,1);
                                    squared_distances_to_crosspoint_j = sum(([lanelet_x_j,lanelet_y_j]-collision_point).^2,2);
                                    [~,idx_closest_two_point_j] = mink(squared_distances_to_crosspoint_j,2,1);
                                    % the endpoint of the curve is the collision point and the adjacent left point is the one among the closest two points with a smaller index  
                                    curve_x_i = [lanelet_x_i(1:min(idx_closest_two_point_i));collision_point(1)]; curve_y_i = [lanelet_y_i(1:min(idx_closest_two_point_i));collision_point(2)];
                                    curve_x_j = [lanelet_x_j(1:min(idx_closest_two_point_j));collision_point(1)]; curve_y_j = [lanelet_y_j(1:min(idx_closest_two_point_j));collision_point(2)];
                                end
    
                                % calculate the arc length from the vehicle's current position to the collision point
                                [distance_to_collision_i, ~, ~, ~, ~] = get_arc_distance_to_endpoint(position_i(1), position_i(2), curve_x_i, curve_y_i);
                                [distance_to_collision_j, ~, ~, ~, ~] = get_arc_distance_to_endpoint(position_j(1), position_j(2), curve_x_j, curve_y_j);
                            else
                                collision_point = lanelet_relationship.point;
                                distance_to_collision_i = norm(position_i-collision_point);
                                distance_to_collision_j = norm(position_j-collision_point);
                            end

                            % determine who is the leader 
                            is_leader = determine_leader_and_follower(veh_i, veh_j, distance_to_collision_i, distance_to_collision_j, is_both_at_intersection, time_enter_intersection, directed_adjacency_old, is_forking_lanelets);

                            time_to_collisionPoint_i = get_the_shortest_time_to_arrive(scenario.mpa,trim_i,distance_to_collision_i,scenario.dt);
                            time_to_collisionPoint_j = get_the_shortest_time_to_arrive(scenario.mpa,trim_j,distance_to_collision_j,scenario.dt);

                            % the first arrived vehicle will wait for the second arrived vehicle to "achieve" a collision.
                            % Here we ignore the fact that the first arrived vehicle should also decelerate to stop at the
                            % point of intersection, which could be added to todo list to improve the exactness.
                            waiting_time = abs(time_to_collisionPoint_i-time_to_collisionPoint_j);
                            % actual STAC
                            STAC = max(time_to_collisionPoint_i,time_to_collisionPoint_j);
                            % adapted STAC
                            STAC_adapted = STAC + waiting_time;                            
                        end

                        % store coupling information
                        coupling_infos(count).STAC_adapted = STAC_adapted;
                        coupling_infos(count).lanelet_relationship = lanelet_relationship.type;
                        if is_leader
%                             disp(['Leader with ID ' num2str(veh_i) ' is coupled with follower with ID ' num2str(veh_j) ' .'])
                            % vehicle_i is the leader
                            coupling_weights(veh_i,veh_j) = weighting_function(STAC_adapted); 
                            coupling_infos(count).idLeader = veh_i; 
                            coupling_infos(count).speedLeader = speed_i;
                            coupling_infos(count).positionLeader = position_i;
                            coupling_infos(count).idFollower = veh_j; 
                            coupling_infos(count).speedFollower = speed_j;
                            coupling_infos(count).positionFollower = position_j;
                        else
%                             disp(['Leader with ID ' num2str(veh_j) ' is coupled with follower with ID ' num2str(veh_i) ' .'])                            
                            % vehicle_j is the leader
                            coupling_weights(veh_j,veh_i) = weighting_function(STAC_adapted); 
                            coupling_infos(count).idLeader = veh_j;
                            coupling_infos(count).speedLeader = speed_j;
                            coupling_infos(count).positionLeader = position_j;
                            coupling_infos(count).idFollower = veh_i;
                            coupling_infos(count).speedFollower = speed_i;
                            coupling_infos(count).positionFollower = position_i;
                        end
                        count = count + 1;
                        stop_flag = true;
                        break
                            
                    end
                    if stop_flag
                        break
                    end
                end
            end   
        end

    end

end


%% local function
function idx_lanelet_pair = find_idx_lanelet_pair(predicted_lanelet_i, predicted_lanelet_j, lanelet_relationships)
    % First try to find predicted_lanelet_i in the field `ID_1` and
    % predicted_lanelet_j in the field `ID_2`. If not find, do it again in
    % the opposite way.
    idx_lanelet_pair = [];
    id_i_j = double([predicted_lanelet_i; predicted_lanelet_j]);
    id_j_i = double([predicted_lanelet_j; predicted_lanelet_i]);
    IDs_1_2 = double([lanelet_relationships.ID_1;lanelet_relationships.ID_2]);

    if isempty(idx_lanelet_pair)
        idx_lanelet_pair = find(all((IDs_1_2-id_i_j)==0));
    end
    if isempty(idx_lanelet_pair)
        idx_lanelet_pair = find(all((IDs_1_2-id_j_i)==0));
    end
end

 
%% local function
function stop_point = get_stop_point_after_travel_certain_distance(position, lanelet, distance_to_travel)
% returns the stop point of the vehicle after travel a certain distance
% along the lanelet
    distances_to_veh = sqrt(sum((lanelet-position).^2,2));
    [~,idx_closest_to_veh] = min(distances_to_veh);
    distances_to_veh(1:idx_closest_to_veh) = inf; % points before vehicle's position are irrelevant
    distances_diff = distance_to_travel - distances_to_veh;
    [~,idx_colsest_to_stop_point] = min(abs(distances_diff));
    vector_direction = lanelet(idx_colsest_to_stop_point,:) - lanelet(idx_colsest_to_stop_point-1,:);
    unit_vector_direction = vector_direction/norm(vector_direction);
    stop_point = lanelet(idx_colsest_to_stop_point,:) + distances_diff(idx_colsest_to_stop_point)*unit_vector_direction;
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
function is_leader = determine_leader_and_follower(veh_i, veh_j, distance_to_collision_i, distance_to_collision_j, is_both_at_intersection, time_enter_intersection, directed_adjacency_old, is_forking_lanelets)
% Determines who is the leader and who is the follower. 
% Vehicle enters the intersection earlier has a higher priority. If both
% vheicle enter the intersection at the same time, their previous
% coupling direction are kept. If they are not coupled at previous time
% step, vehicle who can arrives the point of intersection has a higher
% priority. 

    is_leader = [];
    
    if is_both_at_intersection
        if time_enter_intersection(veh_i) < time_enter_intersection(veh_j)
            is_leader = true; 
        elseif time_enter_intersection(veh_i) > time_enter_intersection(veh_j)
            is_leader = false; 
        else
            is_leader = [];
        end
    end

    if isempty(is_leader)
        % leader-follower relationship is maintained if exists in the last
        % time step
        if directed_adjacency_old(veh_i,veh_j) == 1
            is_leader = true; 
        elseif directed_adjacency_old(veh_j,veh_i) == 1
            is_leader = false;
        else
            % if they are not coupled at previous time step, vehicle closer to the collision point is the leader
            if distance_to_collision_i <= distance_to_collision_j
                is_leader = true; 
            else
                is_leader = false; 
            end

            % for forking lenelts, vehicle closer to the collision point is the follower
            if is_forking_lanelets
                is_leader = ~is_leader;
            end
        end
    end

end

