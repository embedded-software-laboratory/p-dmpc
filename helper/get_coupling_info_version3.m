function [all_veh_at_intersection, coupling_weights, coupling_info, time_enter_intersection] = get_coupling_info_version3(scenario, iter)
% GET_COUPLING_INFO This function estimate the coupling information of
% vehicles. Two vehicles are coupled if their reachable sets in the defined 
% prediction horizon (Hp) overlap. To reduce the checking effort, only the
% reachable sets in the last time step will be checked, which is almost
% sufficient. Heuristic method will be used to estimate the coupling degree
% between vehicles. More concrete,, the so-called "the shortest time to
% achieve a collision (STAC)" will be calculated, whilea a lower STAC means
% a higher coupling degree. 

    nVeh = scenario.nVeh;

    % previous directed adjacency matrix
    directed_adjacency_old = zeros(nVeh,nVeh); 
    if scenario.k > 1
        for i = 1:length([scenario.coupling_info.veh_with_ROW])
            veh_with_ROW = scenario.coupling_info(i).veh_with_ROW;
            veh_without_ROW = scenario.coupling_info(i).veh_without_ROW;
            directed_adjacency_old(veh_with_ROW,veh_without_ROW) = 1;
        end
    end

    % record the time step that each vehicle enters the intersection
    time_enter_intersection = scenario.time_enter_intersection;
    if isempty(time_enter_intersection)
        % this is empty only at the initial time step
        time_enter_intersection = inf*ones(1,nVeh);
    end

    last_veh_at_intersection = scenario.last_veh_at_intersection;

    % vehicles are considered as at the intersection if their distances to
    % the intersection center point is smaller than a certain value
    distances_to_intersection_center = sqrt(sum((iter.x0(:,1:2) - scenario.intersection_center).^2,2));
    all_veh_at_intersection = find(distances_to_intersection_center < scenario.distance_threshold_intersection);

    new_veh_at_intersection = setdiff(all_veh_at_intersection, last_veh_at_intersection);
    if ~isempty(new_veh_at_intersection)
        % disp('debug')
    end

    veh_leave_intersection = setdiff(last_veh_at_intersection,all_veh_at_intersection);
    time_enter_intersection(new_veh_at_intersection) = scenario.k;
    time_enter_intersection(veh_leave_intersection) = inf; % set to inf if vehicle leaves the intersection
    
    coupling_weights = zeros(nVeh,nVeh); % coupling weights of all coupling vehicle pair; higher value indicates stronger coupling
    
    % initialize a struct array to store information about coupling information
    coupling_info = struct('veh_with_ROW',[],'veh_without_ROW',[],'type',[],'speedLeader',[],'speedFollower',[],'positionLeader',[],'positionFollower',[],'STAC_adapted',[],'weight',[]); 
    count = 1;

    state_indices = indices();
    
    for veh_i = 1:(nVeh-1)
        % get the selected vehicle's position, speed, current trim and predicted lanelets 
        speed_i = iter.x0(veh_i, state_indices.speed);
        position_i = [iter.x0(veh_i,state_indices.x), iter.x0(veh_i,state_indices.y)];
        trim_i = iter.trim_indices(veh_i);

        for veh_j = (veh_i+1):nVeh
            speed_j = iter.x0(veh_j, state_indices.speed);
            position_j = [iter.x0(veh_j,state_indices.x), iter.x0(veh_j,state_indices.y)];
            trim_j = iter.trim_indices(veh_j);

            % if two vehicles' reachable sets overlap, they are considered
            % as coupled. To save computation time, only the reachable sets
            % at the last prediction horizon will be checked.
            overlap_reachable_sets = intersect(iter.reachable_sets{veh_i,end}, iter.reachable_sets{veh_j,end});
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
                time_to_collisionPoint_j = get_the_shortest_time_to_arrive(scenario.mpa, trim_j, distance_to_collision_j, scenario.dt);

                % determine leader-follower relationship
                if ismember(veh_i,all_veh_at_intersection) && ismember(veh_j,all_veh_at_intersection)
                    % if both vehicles are at the intersection
                    is_leader = determine_leader_and_follower(veh_i, veh_j, time_enter_intersection, directed_adjacency_old, time_to_collisionPoint_i, time_to_collisionPoint_j);
                else
                    % otherwise vehicle who needs less to arrive the collision point is the leader
                    if time_to_collisionPoint_i <= time_to_collisionPoint_j
                        is_leader = true; 
                    else
                        is_leader = false; 
                    end
                end

                % the first arrived vehicle will wait for the second arrived vehicle to "achieve" a collision.
                % Here we ignore the fact that the first arrived vehicle should also decelerate to stop at the
                % point of intersection, which could be added to todo list to improve the exactness.
                waiting_time = abs(time_to_collisionPoint_i-time_to_collisionPoint_j);
                % actual STAC
                STAC_actual = max(time_to_collisionPoint_i,time_to_collisionPoint_j);
                % adapted STAC
                STAC_adapted = STAC_actual + waiting_time;  

                % store coupling information
                coupling_info(count).STAC_adapted = STAC_adapted;
                coupling_info(count).weight = weighting_function(STAC_adapted);

                if is_leader % vehicle_i is the leader
%                     disp(['Leader with ID ' num2str(veh_i) ' is coupled with follower with ID ' num2str(veh_j) ' .']) 
                    coupling_weights(veh_i,veh_j) = coupling_info(count).weight; 
                    coupling_info(count).veh_with_ROW = veh_i; 
                    coupling_info(count).speedLeader = speed_i;
                    coupling_info(count).positionLeader = position_i;
                    coupling_info(count).veh_without_ROW = veh_j; 
                    coupling_info(count).speedFollower = speed_j;
                    coupling_info(count).positionFollower = position_j;
                else % vehicle_j is the leader
%                     disp(['Leader with ID ' num2str(veh_j) ' is coupled with follower with ID ' num2str(veh_i) ' .'])
                    coupling_weights(veh_j,veh_i) = coupling_info(count).weight; 
                    coupling_info(count).veh_with_ROW = veh_j;
                    coupling_info(count).speedLeader = speed_j;
                    coupling_info(count).positionLeader = position_j;
                    coupling_info(count).veh_without_ROW = veh_i;
                    coupling_info(count).speedFollower = speed_i;
                    coupling_info(count).positionFollower = position_i;
                end
                count = count + 1; 
 
            end   
        end

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

