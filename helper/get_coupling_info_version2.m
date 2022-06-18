function [vehs_at_intersection, coupling_weights, coupling_info, time_enter_intersection] = get_coupling_info_version2(scenario, iter)
% GET_COUPLING_INFO This function estimate the coupling information of
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
        for i = 1:length([scenario.coupling_info.veh_with_ROW])
            veh_with_ROW = scenario.coupling_info(i).veh_with_ROW;
            veh_without_ROW = scenario.coupling_info(i).veh_without_ROW;
            directed_adjacency_old(veh_with_ROW,veh_without_ROW) = 1;
        end
    end

    time_enter_intersection = scenario.time_enter_intersection;
    if isempty(time_enter_intersection)
        % this is empty only at the initial time step
        time_enter_intersection = inf*ones(1,nVeh); % time step when vehicle enters the intersection
    end

    last_vehs_at_intersection = scenario.last_vehs_at_intersection;

    % vehicles are considered as at the intersection if their distances to
    % the intersection center point is smaller than a certain value
    distances_to_center = sqrt(sum((iter.x0(:,1:2) - scenario.intersection_center).^2,2));
    vehs_at_intersection = find(distances_to_center < 1.1);

    new_veh_at_intersection = setdiff(vehs_at_intersection, last_vehs_at_intersection);

    veh_leave_intersection = setdiff(last_vehs_at_intersection,vehs_at_intersection);
    time_enter_intersection(new_veh_at_intersection) = scenario.k;
    time_enter_intersection(veh_leave_intersection) = inf; % set to inf if vehicle leaves the intersection
    
    coupling_weights = zeros(nVeh,nVeh); % coupling weights of all coupling vehicle pair; higher value indicates stronger coupling
    
    % initialize a struct array to store information about coupling information
    % ROW stands for right-of-way
    coupling_info = struct('veh_with_ROW',[],'veh_without_ROW',[],'collision_type',[],'lanelet_relationship',[],'STAC',[]); 
    count = 1;

    state_indices = indices();
    
    for veh_i = 1:(nVeh-1)
        % get the selected vehicle's position, speed, current trim and predicted lanelets 
        position_i = [iter.x0(veh_i,state_indices.x), iter.x0(veh_i,state_indices.y)];
        trim_i = iter.trim_indices(veh_i);
        predicted_lanelets_i = iter.predicted_lanelets{veh_i};

        for veh_j = (veh_i+1):nVeh
            position_j = [iter.x0(veh_j,state_indices.x), iter.x0(veh_j,state_indices.y)];
            trim_j = iter.trim_indices(veh_j);
            predicted_lanelets_j = iter.predicted_lanelets{veh_j};

            overlap_reachable_sets = intersect(iter.reachable_sets{veh_i,end}, iter.reachable_sets{veh_j,end});
            area_overlap = area(overlap_reachable_sets);

            if scenario.k>=62 && veh_i==11
                disp('')
            end
            if area_overlap > 1e-3 % a small threshold to tolerate measurement error of lenelet boundary
                stop_flag = false;
                % the selected two vehicles are considered as coupled if their reachable sets overlap
                for predicted_lanelet_i = predicted_lanelets_i                    
                    for predicted_lanelet_j = predicted_lanelets_j

                        if predicted_lanelet_i==predicted_lanelets_i(end) && predicted_lanelet_j==predicted_lanelets_j(end)
                            is_last_lanelet_pair = true;
                        else
                            is_last_lanelet_pair = false;
                        end

                        veh_length = mean([scenario.vehicles(veh_i).Length, scenario.vehicles(veh_i).Length]);

                        [lanelet_type,collision_type,is_continue,lanelet_relationship,is_find_lanelet_relationship,lanelet] = ...
                            get_collision_and_lanelet_type(position_i,position_j,predicted_lanelet_i,predicted_lanelet_j,is_last_lanelet_pair,lanelets,lanelet_relationships,overlap_reachable_sets,veh_length);

                        if is_continue
                            continue
                        end

                        % check if both vehicles are at the intersection 
                        if ismember(veh_i,vehs_at_intersection) && ismember(veh_j,vehs_at_intersection)
                            is_both_at_intersection = true;
                        else
                            is_both_at_intersection = false;
                        end

                        % estimate the STAC and coupling weight
                        if collision_type.is_rear_end
                            % If the same lanelet or successive (relationship type 1) lanelets or adjacent left/right lanelets (relationship type 2) -> rear-end collision
                            coupling_info(count).collision_type = CollisionType.type_1; % rear-end collision

                            % the same lanelet ot successive lanelet or forking lanelets
                            collision_point = lanelet_relationship.point;
                            distance_to_collision_i = norm(position_i-collision_point,2);
                            distance_to_collision_j = norm(position_j-collision_point,2);
                            
                            % determine who is the leader 
                            is_leader = determine_who_has_ROW(veh_i, veh_j, distance_to_collision_i, distance_to_collision_j, is_both_at_intersection, time_enter_intersection, directed_adjacency_old, lanelet_type.is_forking);
                            
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

                        elseif collision_type.is_side_impact
                            coupling_info(count).collision_type = CollisionType.type_2; % side-impact collision

                            % For side-impact collision, we calculate the adapted STAC, which is the sum of the actual STAC and a waiting time. 
                            % The calculation procedure is the following: Firstly, the actual STAC is the shortest time needed to achieve a 
                            % collision at the point of intersection. This is calculated by letting both vehicles take a full acceleration to 
                            % arrive at the point of intersection. The first arrived vehicle has to stop at the point of intersection and wait 
                            % for the second arrived vehicle, where the waiting time is the second part of the adapted STAC.
                            collision_point = lanelet_relationship.point;
                            if is_find_lanelet_relationship
                                % from a curve to calculate the arc diatance between vehicle's current position and the collision point, 
                                % which starts from the starting point of the lanelet and ends at the collision point 
                                if lanelet_type.is_merging 
                                    % collision point of the merging lanelets is both lanelets' endpoint, thus the target curve is the whole lanelet
                                    curve_x_i = lanelet.x_i; curve_y_i = lanelet.y_i;
                                    curve_x_j = lanelet.x_j; curve_y_j = lanelet.y_j;
                                elseif lanelet_type.is_intersecting 
                                    % Collision point of the intersecting lanelets is the crosspoint, which could be in the middle of the lanelets
                                    % First, find the two closest points to the crosspoint on the lanelet
                                    squared_distances_to_crosspoint_i = sum(([lanelet.x_i,lanelet.y_i]-collision_point).^2,2);
                                    [~,idx_closest_two_point_i] = mink(squared_distances_to_crosspoint_i,2,1);
                                    squared_distances_to_crosspoint_j = sum(([lanelet.x_j,lanelet.y_j]-collision_point).^2,2);
                                    [~,idx_closest_two_point_j] = mink(squared_distances_to_crosspoint_j,2,1);
                                    % the endpoint of the curve is the collision point and the adjacent left point is the one among the closest two points with a smaller index  
                                    curve_x_i = [lanelet.x_i(1:min(idx_closest_two_point_i));collision_point(1)]; curve_y_i = [lanelet.y_i(1:min(idx_closest_two_point_i));collision_point(2)];
                                    curve_x_j = [lanelet.x_j(1:min(idx_closest_two_point_j));collision_point(1)]; curve_y_j = [lanelet.y_j(1:min(idx_closest_two_point_j));collision_point(2)];
                                end
    
                                % calculate the arc length from the vehicle's current position to the collision point
                                [distance_to_collision_i, ~, ~, ~, ~, ~] = get_arc_distance_to_endpoint(position_i(1), position_i(2), curve_x_i, curve_y_i);
                                [distance_to_collision_j, ~, ~, ~, ~, ~] = get_arc_distance_to_endpoint(position_j(1), position_j(2), curve_x_j, curve_y_j);
                            else
                                distance_to_collision_i = norm(position_i-collision_point);
                                distance_to_collision_j = norm(position_j-collision_point);
                            end

                            % determine who is the leader 
                            is_leader = determine_who_has_ROW(veh_i, veh_j, distance_to_collision_i, distance_to_collision_j, is_both_at_intersection, time_enter_intersection, directed_adjacency_old, lanelet_type.is_forking);

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
                        coupling_info(count).STAC = STAC;
                        coupling_info(count).lanelet_relationship = lanelet_relationship.type;
                        if is_leader
                            % vehicle_i has the right-of-way
                            coupling_weights(veh_i,veh_j) = weighting_function(STAC_adapted); 
                            coupling_info(count).veh_with_ROW = veh_i;  
                            coupling_info(count).veh_without_ROW = veh_j; 
                        else
                            % vehicle_j has the right-of-way
                            coupling_weights(veh_j,veh_i) = weighting_function(STAC_adapted); 
                            coupling_info(count).veh_with_ROW = veh_j;
                            coupling_info(count).veh_without_ROW = veh_i;
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

    % If two vehicles drive successively and are very close to each other,
    % the back vehicle will inherit the right-of-way of the front vehicle 
    [coupling_weights, coupling_info, time_enter_intersection] = inherit_right_of_way(coupling_weights, coupling_info, time_enter_intersection, iter.predicted_lanelets);
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
        idx_lanelet_pair = find(all((IDs_1_2-id_i_j)==0,1));
    end
    if isempty(idx_lanelet_pair)
        idx_lanelet_pair = find(all((IDs_1_2-id_j_i)==0,1));
    end
end

 
%% local function
function stop_point = get_stop_point_after_travel_certain_distance(position, lanelet, distance_to_travel)
% returns the stop point of the vehicle after travel a certain distance
% along the lanelet
    distances_to_veh = sqrt(sum((lanelet-position).^2,2));
    [~,idx_closest_to_veh] = min(distances_to_veh);
    distances_to_veh(1:idx_closest_to_veh-1) = inf; % points before vehicle's position are irrelevant
    distances_diff = distance_to_travel - distances_to_veh;
    [~,idx_colsest_to_stop_point] = min(abs(distances_diff));
    idx_colsest_to_stop_point = max(2,idx_colsest_to_stop_point); % index is at least 2
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
function has_ROW = determine_who_has_ROW(veh_i, veh_j, distance_to_collision_i, distance_to_collision_j, is_both_at_intersection, time_enter_intersection, directed_adjacency_old, is_forking_lanelets)
% Determine whos has the right-of-way. 
% Vehicle enters the intersection earlier has the right-of-way. If both
% vheicle enter the intersection at the same time, right-of-way is given to
% vehicle which has the right-of-way in the last time step. If they are not
% coupled at previous time step, vehicle who can arrive the collision point
% earilier has the right-of-way.

    has_ROW = [];
    
    if is_both_at_intersection
        if time_enter_intersection(veh_i) < time_enter_intersection(veh_j)
            has_ROW = true; 
        elseif time_enter_intersection(veh_i) > time_enter_intersection(veh_j)
            has_ROW = false; 
        else
            has_ROW = [];
        end
    end

    if isempty(has_ROW)
        % leader-follower relationship is maintained if exists in the last
        % time step
        if directed_adjacency_old(veh_i,veh_j) == 1
            has_ROW = true; 
        elseif directed_adjacency_old(veh_j,veh_i) == 1
            has_ROW = false;
        else
            % if they are not coupled at previous time step, vehicle closer to the collision point is the leader
            if distance_to_collision_i <= distance_to_collision_j
                has_ROW = true; 
            else
                has_ROW = false; 
            end

            % for forking lenelts, vehicle closer to the collision point is the follower
            if is_forking_lanelets
                has_ROW = ~has_ROW;
            end
        end
    end
end


%% local function
function [coupling_weights, coupling_info, time_enter_intersection] = inherit_right_of_way(coupling_weights, coupling_info, time_enter_intersection, predicted_lanelets)
% If two vehicles have the same reference path, drive successively and are very close to each other,
% the back vehicle will inherit the right-of-way of the front vehicle  

    nVeh = length(coupling_weights);

    coupling_weights_adjusted = coupling_weights; % make a copy
    
    STAC_threshold = 0.6; % vehicles are very close if they can achieve a collision in a short time
    find_small_STAC = [coupling_info.STAC] < STAC_threshold; % find coupling pairs with a small STAC

    if ~any(find_small_STAC)
        return
    end

    all_coupling_pairs = [coupling_info.veh_with_ROW;coupling_info.veh_without_ROW];
    for iVeh = 1:nVeh
        vehs_inherit = []; % all target vehicles that could be inherited 
        predicted_lanelet_i = predicted_lanelets{iVeh};

        % get the first two predicted lanelets
        if length(predicted_lanelet_i) < 2
            % at least two predicted lanelets are needed to check if reference paths is the same or not  
            continue
        else
            % only take the first two predicted lanelets
            predicted_lanelet_i = predicted_lanelet_i(1:2);
        end

        veh_with_ROW = iVeh; % initialize variable to find all target front vehicles 
        while true
            % iteratively find all front vehicles, i.e., front vehicle could also inherit another vehicle in front 
            find_vehs_with_ROW = [coupling_info.veh_without_ROW]==veh_with_ROW;
            find_front_vehs = all([find_vehs_with_ROW; find_small_STAC],1);
            front_vehs_indices = find(find_front_vehs);
            front_vehs_to_delete = [];

            % only keep those front vehicles that have the same reference path as the selected vehicle
            for j = 1:length(front_vehs_indices)
                j_coupling = front_vehs_indices(j);
                veh_j = coupling_info(j_coupling).veh_with_ROW;

                predicted_lanelet_j = predicted_lanelets{veh_j};

                % get the first two predicted lanelets
                if length(predicted_lanelet_j) < 2
                    front_vehs_to_delete(end+1) = j;
                else
                    predicted_lanelet_j = predicted_lanelet_j(1:2);
                end
    
                same_lanelets = ismember(predicted_lanelet_i,predicted_lanelet_j);
                num_same_lanelets = sum(same_lanelets);

                if num_same_lanelets == 0
                    % if no same lanelet: not the same reference path
                    front_vehs_to_delete(end+1) = j;
                elseif num_same_lanelets == 1
                    % if only one lanelet is the same, it must be the
                    % second predicted lanelet of the back vehicle and the first
                    % predicted lanelet of the front vehicle
                    if predicted_lanelet_i(2) ~= predicted_lanelet_j(1)
                        front_vehs_to_delete(end+1) = j;
                    end
                end
            end

            front_vehs_indices(front_vehs_to_delete) = [];
            
            if isempty(front_vehs_indices)
                % if no such front vehicle of the front vehicle exists
                break
            end

            % choose the one which has the minimum STAC with the selected
            % vehicle
            [~,idx_min_STAC] = min([coupling_info(front_vehs_indices).STAC]);
            i_coupling = front_vehs_indices(idx_min_STAC);
            veh_with_ROW = coupling_info(i_coupling).veh_with_ROW;
            
            if veh_with_ROW == iVeh
                % in case of coupling circle
               break
            end
            vehs_inherit(end+1) = veh_with_ROW;
        end

        if isempty(vehs_inherit)
            continue
        end

        % inherit the time entering the intersection
        time_enter_intersection(iVeh) = min(time_enter_intersection(vehs_inherit));

        % inherit the right-of-way from the front vehicle
        % find all coupled vehicles that have side-impact collision possibility with the selected vehicle 
        vehs_with_ROW_j = find(coupling_weights(:,iVeh) ~= 0);

        for veh_with_ROW_j = vehs_with_ROW_j(:)'
            j_coupling = find(all(all_coupling_pairs-[veh_with_ROW_j;iVeh]==0,1));

            % check if the coupled vehicle has side-impact collision
            % possibility both with the selected vehicle and the vehicle to be inherited 
            if strcmp(coupling_info(j_coupling).collision_type, CollisionType.type_2)
                % side-impact collision with the selected vehicle
                find_vehs_to_inherit = coupling_weights(vehs_inherit,veh_with_ROW_j)~=0;
                find_vehs_to_inherit = vehs_inherit(find_vehs_to_inherit);
                for veh_to_inherit = find_vehs_to_inherit(:)'
                    k_coupling = all(all_coupling_pairs-[veh_to_inherit;veh_with_ROW_j]==0,1);
                    if strcmp(coupling_info(k_coupling).collision_type, CollisionType.type_2)
                        % side-impact collision with the vehicle to be inherited 
                        disp(['After inheriting right-of-way from vehicle ' num2str(veh_to_inherit) ', ' num2str(iVeh) ' now has the right-of-way over ' num2str(veh_with_ROW_j) '.'])
                        % update coupling direction
                        coupling_weights_adjusted(veh_with_ROW_j,iVeh) = 0;
                        coupling_weights_adjusted(iVeh,veh_with_ROW_j) = coupling_weights(veh_with_ROW_j,iVeh);
                        
                        % update coupling information
                        [coupling_info(j_coupling).veh_with_ROW, coupling_info(j_coupling).veh_without_ROW] = ...
                            swap(coupling_info(j_coupling).veh_with_ROW, coupling_info(j_coupling).veh_without_ROW);
                    end
                end

                
            end
        end
        
    end

    % update coupling weights
    coupling_weights = coupling_weights_adjusted;
end

%% local function
function [lanelet_type,collision_type,is_continue,lanelet_relationship,is_find_lanelet_relationship,lanelet] = get_collision_and_lanelet_type(position_i,position_j,predicted_lanelet_i,predicted_lanelet_j,is_last_lanelet_pair,lanelets,lanelet_relationships,overlap_reachable_sets,veh_length)

    % initialize
    lanelet_type = struct('is_same',false,'is_forking',false,'is_merging',false,'is_intersecting',false,'is_successive',false);
    collision_type = struct('is_rear_end',false,'is_side_impact',false);
    is_continue = false;
    lanelet_relationship = struct('ID_1',[],'ID_2',[],'type',[],'point',[]); 
    is_find_lanelet_relationship = false;
    lanelet = struct('x_i',[],'y_i',[],'x_j',[],'y_j',[]);
    
    % center line of the lanelet
    lanelet.x_i = lanelets{predicted_lanelet_i}(:,LaneletInfo.cx); 
    lanelet.y_i = lanelets{predicted_lanelet_i}(:,LaneletInfo.cy);
    lanelet.x_j = lanelets{predicted_lanelet_j}(:,LaneletInfo.cx); 
    lanelet.y_j = lanelets{predicted_lanelet_j}(:,LaneletInfo.cy);

    % get the relationship of their predicted lanelets and the possible collision type 
    if predicted_lanelet_i == predicted_lanelet_j
        lanelet_type.is_same = true; % the same lanelet
        % if the same lanelet: rear-end collision
        collision_type.is_rear_end = true;
        % create information of lanelet relationship
        lanelet_relationship.type = LaneletRelationshipType.type_6; % same lanelet
        lanelet_relationship.point = [lanelet.x_i(end),lanelet.y_i(end)]; % store the endpoint of the lanelet
    else
        % find if there exists lanelet pair that has a certain relationship in the struct array `lanelet_relationships`
        % NOTE that only adjacent lanelet pairs with certain relationships will be stored in `lanelet_relationships`
        idx_lanelet_pair = find_idx_lanelet_pair(predicted_lanelet_i,predicted_lanelet_j,lanelet_relationships);
        if ~isempty(idx_lanelet_pair)
            is_find_lanelet_relationship = true;
            lanelet_relationship = lanelet_relationships(idx_lanelet_pair);

            % If forking lanelets (relationship type 4) or successive (relationship type 1) lanelets or 
            % adjacent left/right lanelets (relationship type 2) -> rear-end collision
            lanelet_type.is_successive = strcmp(lanelet_relationship.type, LaneletRelationshipType.type_1);
            lanelet_type.is_merging = strcmp(lanelet_relationship.type, LaneletRelationshipType.type_3);
            lanelet_type.is_forking = strcmp(lanelet_relationship.type, LaneletRelationshipType.type_4);
            lanelet_type.is_intersecting = strcmp(lanelet_relationship.type, LaneletRelationshipType.type_5);
            lanelet_type.is_same = strcmp(lanelet_relationship.type, LaneletRelationshipType.type_6);

            if lanelet_type.is_successive || lanelet_type.is_same || lanelet_type.is_forking
                collision_type.is_rear_end = true;
            elseif lanelet_type.is_intersecting
                % only side-impact could happen at intersection lanelets
                collision_type.is_side_impact = true;
            elseif lanelet_type.is_merging
                % both side-impact and rear-end collision could happen at mering lanelets
                % 1. rear-end collision: if the difference between their distances to collision point is larger than a certain velue (such as 1.5*vehicleLength)
                % 2. otherwise side-impact collision

                % get arc distance to collision point 
                [arc_distance_i, ~, ~, ~, ~, ~] = get_arc_distance_to_endpoint(position_i(1), position_i(2), lanelet.x_j, lanelet.y_j);
                [arc_distance_j, ~, ~, ~, ~, ~] = get_arc_distance_to_endpoint(position_j(1), position_j(2), lanelet.x_i, lanelet.y_i); 

                safety_factor = 1.5;
                if abs(arc_distance_i-arc_distance_j) > safety_factor*veh_length
%                 if norm([x_projected_i,y_projected_i]-position_j) > wheelbase_mean && norm([x_projected_j,y_projected_j]-position_i) > wheelbase_mean
                    collision_type.is_rear_end = true;
                else
                    collision_type.is_side_impact = true;
                end  
            else
                error('Unknown lanelet relationship.')
            end
        else
            if is_last_lanelet_pair
                % If no lanelet relationship is found until the last predicted lanelet pair
                % 1. Set the center point of the overlapping area as collision point
                % 2. Set lanelet relationship to be intersecting
                % 3. Set collision type to be side-impact collision (normally lanelet relationship will always be found for rear-end collision)
%                 disp(['No lanelet relationship can be found for coupled vehicle ' num2str(veh_i) ' and ' num2str(veh_j) '.' newline...
%                     'Centroid of the overlapping area of their reachable sets will be used as collision point.'])
                
                collision_type.is_side_impact = true;

                lanelet_relationship.type = LaneletRelationshipType.type_5; % intersecting lanelets
                [x_centroid,y_centroid] = centroid(overlap_reachable_sets);
                lanelet_relationship.point = [x_centroid,y_centroid];
            else
                % jump to the next predicted lanelet
                is_continue = true;
            end
        end
    end

end

