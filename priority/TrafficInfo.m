classdef TrafficInfo
    % TrafficInfo Traffic information, such as which vehicle is coupled
    % with which vehicle, how strong are they coupled, which vehicles are
    % at the intersection and when did they enter the intersection

    % Some terms:
        % ACP: Assumed collision point.

        % LCP: Lanelet critical point.
            % LCP(merging-adjacent) = merging point;
            % LCP(forking-adjacent) = forking point;
            % LCP(crossing-adjacent) = crossing point;
            % LCP(left/right-adjacent) = end point of their shared bound
            % LCP(same) = end point of the centerline
    
        % STAC: The shortest time to achieve a collision. For rear-end collisions, calculated by
        % letting the front vehicle take an emergency braking and the rear
        % vehicle take a full acceleration. For side-impact collisions,
        % calculated by letting both vehicles take a full acceleration.

        % Waiting time: Only used in side-impact collisions. This is the
        % time that the earilier-arrived vehicle needs to wait for the
        % later-arrived vehicle to achieve a collision.
        
    properties
        coupling_weights            % (nVeh-by-nVeh matrix) coupling weights of all coupling vehicle pair; higher value indicates stronger coupling
        coupling_weights_optimal    % "optimal" coupling weights
        directed_adjacency_old      % (nVeh-by-nVeh matrix) previous directed adjacency matrix
        coupling_info               % (struct, length equals to number of coupling pair) information of each two coupled vehicles
        vehs_at_intersection        % (vector) vehicles at intersection
        time_enter_intersection     % (vector) time at which vehicle enter intersection. Value is inf if vehicle is not at intersection
        nVeh                        % total number of vehicles
        priority_option             % right_of_way_priority/random_priority/constant_priority
    end

    properties (Constant)
        distance_to_CP_threshold = 1.2;         % vehicles are considered as at the intersecrion if their distances to the center point of intersection is smaller than this value
        STAC_threshold = 0.5;                   % vehicles are considered as very close if they can achieve a collision in less than this time
        sensitive_factor = 1;                   % sensitive factor used to calculate the coupling weights. The bigger, the more sensitive to the STAC. Default value 1.
        waiting_time_factor = 0.25;             % how important should the waiting time in side-impact collisions be 
        side_impact_weight_scale_factor = 1.25; % side-impact collision is harder to avoid, so we scale its weight
    end

    methods
        function obj = TrafficInfo(scenario, iter)
            obj.nVeh = scenario.options.amount;
            obj.priority_option = scenario.options.priority;
            
            % initialize
            obj.directed_adjacency_old = zeros(obj.nVeh,obj.nVeh);
            obj.coupling_weights = zeros(obj.nVeh,obj.nVeh); 
            obj.coupling_weights_optimal = zeros(obj.nVeh,obj.nVeh); 
            % ROW stands for right-of-way
            obj.coupling_info = struct('veh_with_ROW',[],'veh_without_ROW',[],'collision_type',[],...
                'lanelet_relationship',[],'STAC',[],'is_at_intersection',[],'is_move_side_by_side',[],'is_ignored',[]); 

            % estimate traffic information
            obj = obj.estimate_traffic_status(scenario,iter);
        end
        
        function obj = estimate_traffic_status(obj, scenario, iter)
            % This function estimates traffic status

            % get previous coupling matrix
            if scenario.k>1
                for i = 1:length([scenario.coupling_info.veh_with_ROW])
                    veh_with_ROW = scenario.coupling_info(i).veh_with_ROW;
                    veh_without_ROW = scenario.coupling_info(i).veh_without_ROW;
                    obj.directed_adjacency_old(veh_with_ROW,veh_without_ROW) = 1;
                end
            end
        
            obj = obj.update_vehs_at_intersection(scenario,iter.x0);

            for veh_i = 1:(obj.nVeh-1)
                for veh_j = (veh_i+1):obj.nVeh
                    % TODO: here was the reachable set coupling check, use adjacency which is need either way (to avoid double calculation)
                    if scenario.adjacency(veh_i,veh_j)
                        % the selected two vehicles are considered as coupled
                        if ~scenario.options.consider_RSS &&((scenario.vehicle_ids(veh_i) == scenario.manual_vehicle_id && scenario.options.firstManualVehicleMode == 2) ...
                            || (scenario.vehicle_ids(veh_i) == scenario.second_manual_vehicle_id && scenario.options.secondManualVehicleMode == 2))
                            % Naive approach: not necessary to determine collision point and ROW, as manual vehicle has highest priority
                            obj.coupling_weights(veh_i,veh_j) = 1;
                            continue
                        else
                            overlap_reachable_sets = intersect(iter.reachable_sets{veh_i,end}, iter.reachable_sets{veh_j,end});
                            obj = obj.get_coupling_info(scenario,iter,veh_i,veh_j,overlap_reachable_sets);
                        end
                    end  
                end
            end
        
            % If two vehicles drive successively and are very close to each other,
            % the back vehicle will inherit the right-of-way of the front vehicle
            if scenario.options.isAllowInheritROW
                obj = obj.inherit_right_of_way(iter.predicted_lanelets);
            end
        end
    end

    methods (Access = private)
        function obj = update_vehs_at_intersection(obj,scenario,x0)
            % This function updates the vehicles at intersection and the time
            % they entered the intersection
            obj.time_enter_intersection = scenario.time_enter_intersection;
            if isempty(obj.time_enter_intersection)
                % this is empty only at the initial time step
                obj.time_enter_intersection = inf*ones(1,obj.nVeh); % time step when vehicle enters the intersection
            end
                
            % vehicles are considered as at the intersection if their distances to
            % the intersection center point is smaller than a certain value
            distances_to_center = sqrt(sum((x0(:,1:2) - scenario.intersection_center).^2,2));
            obj.vehs_at_intersection = find(distances_to_center < obj.distance_to_CP_threshold);
        
            new_veh_at_intersection = setdiff(obj.vehs_at_intersection, scenario.last_vehs_at_intersection);
        
            veh_leave_intersection = setdiff(scenario.last_vehs_at_intersection, obj.vehs_at_intersection);
            obj.time_enter_intersection(new_veh_at_intersection) = scenario.k;
            obj.time_enter_intersection(veh_leave_intersection) = inf; % set to inf if vehicle leaves the intersection
        end


        function obj = get_coupling_info(obj,scenario,iter,veh_i,veh_j,overlap_reachable_sets)
            % This function estimate the information of each two coupled vehicles
            stop_flag = false;
            count = length([obj.coupling_info.veh_with_ROW]) + 1; % for the next coupling infomation 

            veh_info_i = struct('ID',[],'position',[],'yaw',[],'trim',[],'predicted_lanelet',[],'length',[],...
                'lanelet_x',[],'lanelet_y',[]);
            veh_info_i.ID = veh_i;
            veh_info_i.position = [iter.x0(veh_i,indices().x), iter.x0(veh_i,indices().y)];
            veh_info_i.yaw = iter.x0(veh_i,indices().heading);
            veh_info_i.trim = iter.trim_indices(veh_i);
            veh_info_i.length = scenario.vehicles(veh_i).Length;
            predicted_lanelets_i = iter.predicted_lanelets{veh_i};

            veh_info_j = struct('ID',[],'position',[],'yaw',[],'trim',[],'predicted_lanelet',[],'length',[],...
                'lanelet_x',[],'lanelet_y',[]);
            veh_info_j.ID = veh_j;
            veh_info_j.position = [iter.x0(veh_j,indices().x), iter.x0(veh_j,indices().y)];
            veh_info_j.yaw = iter.x0(veh_j,indices().heading);
            veh_info_j.trim = iter.trim_indices(veh_j);
            veh_info_j.length = scenario.vehicles(veh_j).Length;
            predicted_lanelets_j = iter.predicted_lanelets{veh_j};

            for pred_lan_i = predicted_lanelets_i                    
                for pred_lan_j = predicted_lanelets_j
    
                    veh_info_i.predicted_lanelet = pred_lan_i;
                    veh_info_j.predicted_lanelet = pred_lan_j;

                    if pred_lan_i==predicted_lanelets_i(end) && pred_lan_j==predicted_lanelets_j(end)
                        is_last_lan_pair = true;
                    else
                        is_last_lan_pair = false;
                    end
                        
                    % center line of the lanelet
                    veh_info_i.lanelet_x = scenario.lanelets{pred_lan_i}(:,LaneletInfo.cx); 
                    veh_info_i.lanelet_y = scenario.lanelets{pred_lan_i}(:,LaneletInfo.cy);
                    veh_info_j.lanelet_x = scenario.lanelets{pred_lan_j}(:,LaneletInfo.cx); 
                    veh_info_j.lanelet_y = scenario.lanelets{pred_lan_j}(:,LaneletInfo.cy);

                    [lanelet_type,collision_type,is_continue,lanelet_relationship,is_find_lanelet_relationship,is_move_side_by_side] = ...
                        obj.get_collision_and_lanelet_type(veh_info_i,veh_info_j,is_last_lan_pair,scenario.lanelet_relationships,overlap_reachable_sets, scenario.options.is_mixed_traffic);

                    if is_continue
                        continue
                    end
    
                    % check if both vehicles are at the intersection 
                    if ismember(veh_i,obj.vehs_at_intersection) && ismember(veh_j,obj.vehs_at_intersection)
                        obj.coupling_info(count).is_at_intersection = true;
                    else
                        obj.coupling_info(count).is_at_intersection = false;
                    end
    
                    % current distance between two vehicles
                    distance_two_vehs = norm(veh_info_i.position-veh_info_j.position,2);
                    assumed_collision_point = lanelet_relationship.point;

                    distance_to_collision_i = norm(veh_info_i.position-assumed_collision_point);
                    distance_to_collision_j = norm(veh_info_j.position-assumed_collision_point);

                    if collision_type.is_rear_end
                        obj.coupling_info(count).collision_type = CollisionType.type_1;
                        % If two vehicles has a rear-end collision possibility, they STAC (shortest time to achieve a collision) is the TTC (time to catch) 
                        has_ROW = obj.determine_who_has_ROW(veh_i, veh_j, distance_to_collision_i, distance_to_collision_j, obj.coupling_info(count).is_at_intersection, lanelet_type.is_forking, scenario.random_seed, scenario, iter);
                        % Calculate the shortest time to achieve a collision
                        if has_ROW               
                            [STAC, waiting_time, ~, ~] = get_the_shortest_time_to_catch(scenario.mpa, veh_info_i.trim, veh_info_j.trim, distance_two_vehs, scenario.options.dt);
                        else
                            [STAC, waiting_time, ~, ~] = get_the_shortest_time_to_catch(scenario.mpa, veh_info_j.trim, veh_info_i.trim, distance_two_vehs, scenario.options.dt);
                        end
%                         waiting_time = 0; % in rear-end collision, the waiting time is not needed
                    else
                        obj.coupling_info(count).collision_type = CollisionType.type_2; % side-impact collision
                        % If two vehicles has a side-impact collision possibility, check if they move in parallel
                        if is_move_side_by_side
                            has_ROW = obj.determine_who_has_ROW(veh_i, veh_j, distance_to_collision_i, distance_to_collision_j, obj.coupling_info(count).is_at_intersection, lanelet_type.is_forking, scenario.random_seed, scenario, iter);
                            STAC = distance_two_vehs/2/max([scenario.mpa.trims.speed])*2;
                            waiting_time = 0;
                        else
                            time_to_collision_point_i = get_the_shortest_time_to_arrive(scenario.mpa,veh_info_i.trim,distance_to_collision_i,scenario.options.dt);
                            time_to_collision_point_j = get_the_shortest_time_to_arrive(scenario.mpa,veh_info_j.trim,distance_to_collision_j,scenario.options.dt);
                            % determine which vehicle has the ROW 
                            % trick: a shorter time to collision point corresponds to a shorter distance to collision point, thus no code adaption is need
                            has_ROW = obj.determine_who_has_ROW(veh_i, veh_j, time_to_collision_point_i, time_to_collision_point_j, obj.coupling_info(count).is_at_intersection, lanelet_type.is_forking, scenario.random_seed, scenario, iter);
                            
                            STAC = max(time_to_collision_point_i,time_to_collision_point_j);
                            waiting_time = abs(time_to_collision_point_i-time_to_collision_point_j);
                        end

                    end

                    STAC_adapted = STAC + obj.waiting_time_factor*waiting_time; % adapted STAC
    
                    % store coupling information
                    obj.coupling_info(count).is_move_side_by_side = is_move_side_by_side; 
                    obj.coupling_info(count).STAC = STAC;
                    obj.coupling_info(count).lanelet_relationship = lanelet_relationship.type;
                    obj.coupling_info(count).is_ignored = false; % whether the coupling is ignored  

                    if has_ROW
                        veh_with_ROW = veh_i; veh_without_ROW = veh_j;
                    else
                        veh_with_ROW = veh_j; veh_without_ROW = veh_i;
                    end
                    
                    switch scenario.options.coupling_weight_mode
                        case 'STAC'
                            obj.coupling_weights(veh_with_ROW,veh_without_ROW) = obj.weighting_function(STAC_adapted, obj.sensitive_factor);
                            if collision_type.is_side_impact
                                % As side-impact collision is harder to avoid, we add more weight to it
                                obj.coupling_weights(veh_with_ROW,veh_without_ROW) = obj.side_impact_weight_scale_factor*obj.coupling_weights(veh_with_ROW,veh_without_ROW);
                            end
                        case 'random'
                            obj.coupling_weights(veh_with_ROW,veh_without_ROW) = rand(scenario.random_seed,1);
                        case 'constant'
                            obj.coupling_weights(veh_with_ROW,veh_without_ROW) = 0.5;
                        case 'optimal'
                            obj.coupling_weights(veh_with_ROW,veh_without_ROW) = get_optimal_coupling_weight(scenario,iter,veh_with_ROW,veh_without_ROW);
                    end

                    obj.coupling_info(count).veh_with_ROW = veh_with_ROW;  
                    obj.coupling_info(count).veh_without_ROW = veh_without_ROW; 

                    stop_flag = true;
                    break
                end

                if stop_flag
                    break % go to the next pair of vehicles
                end
            end
        end


        function [lanelet_type,collision_type,is_continue,lanelet_relationship,is_find_lanelet_relationship,is_move_side_by_side] = ...
                get_collision_and_lanelet_type(obj,veh_info_i,veh_info_j,is_last_lan_pair,lanelet_relationships,overlap_reachable_sets, is_mixed_traffic)
        
            % Initialize variables
            lanelet_type = struct('is_same',false,'is_left_or_right',false,'is_forking',false,'is_merging',false,'is_crossing',false,'is_longitudinal',false);
            collision_type = struct('is_rear_end',false,'is_side_impact',false);
            is_continue = false; % whether to go to check the next lanelet pair
            lanelet_relationship = struct('type',[],'point',[]); 
            is_find_lanelet_relationship = false;
            is_move_side_by_side = false; % whether two vehicles move in parallel
        
            % Get lanelet relationship
            if veh_info_i.predicted_lanelet == veh_info_j.predicted_lanelet
                lanelet_type.is_same = true; % the same lanelet
                % Create information of lanelet relationship
                lanelet_relationship.type = LaneletRelationshipType.type_6; % same lanelet
                lanelet_relationship.point = [veh_info_i.lanelet_x(end),veh_info_i.lanelet_y(end)]; % store the endpoint of the lanelet
            else
                % Find if there exists lanelet pair that has a certain relationship in the struct array `lanelet_relationships`
                % NOTE that only adjacent lanelets will be stored in
                % `lanelet_relationships`, and the index should start from the smaller one 
                if veh_info_i.predicted_lanelet < veh_info_j.predicted_lanelet
                    lanelet_relationship_tmp = lanelet_relationships{veh_info_i.predicted_lanelet,veh_info_j.predicted_lanelet};
                else
                    lanelet_relationship_tmp = lanelet_relationships{veh_info_j.predicted_lanelet,veh_info_i.predicted_lanelet};
                end

                if ~isempty(lanelet_relationship_tmp)
                    is_find_lanelet_relationship = true;
                    lanelet_relationship = lanelet_relationship_tmp;
                else
                    if is_last_lan_pair
                        % If no lanelet relationship is found until the last predicted lanelet pair
                        % 1. Set the center point of the overlapping area as the assumed collision point
                        % 2. Set lanelet relationship to crossing-adjacent  
                        % 3. Set collision type to side-impact collision
                        % (lanelet relationship could not found when centerlines of lanelets at intersection does not intersect, but their boundaries intersect)
                        lanelet_relationship.type = LaneletRelationshipType.type_5; % crossing lanelets
                        [x_centroid,y_centroid] = centroid(overlap_reachable_sets);
                        lanelet_relationship.point = [x_centroid,y_centroid];
                    else
                        % Jump to the next predicted lanelet
                        is_continue = true;
                        return
                    end
                end
            end

            % Get lanelet_type based on lanelet_relationship
            lanelet_type.is_longitudinal = strcmp(lanelet_relationship.type, LaneletRelationshipType.type_1);
            lanelet_type.is_left_or_right = strcmp(lanelet_relationship.type, LaneletRelationshipType.type_2);
            lanelet_type.is_merging = strcmp(lanelet_relationship.type, LaneletRelationshipType.type_3);
            lanelet_type.is_forking = strcmp(lanelet_relationship.type, LaneletRelationshipType.type_4);
            lanelet_type.is_crossing = strcmp(lanelet_relationship.type, LaneletRelationshipType.type_5);
            lanelet_type.is_same = strcmp(lanelet_relationship.type, LaneletRelationshipType.type_6);

            % Get collision_type type based on lanelet_type
            if lanelet_type.is_same || lanelet_type.is_longitudinal || lanelet_type.is_left_or_right || lanelet_type.is_forking
                % For two vehicles at the same, longitudinal-, left/right-, or forking-adjacent lanelets, both two collision types are possible: 
                % 1. Side-impact collision: if they move in parallel
                % 2. Rear-end collision: otherwise
                if obj.check_if_move_parallel(veh_info_i.position,veh_info_j.position,veh_info_i.yaw,veh_info_j.yaw,veh_info_i.length,veh_info_j.length)
                    collision_type.is_side_impact = true;
                    is_move_side_by_side = true;
                else
                    collision_type.is_rear_end = true;
                end
            elseif lanelet_type.is_crossing
                % Only side-impact collision is possible at corssing-adjacent lanelets
                collision_type.is_side_impact = true;
                % The lanelet critical point (LCP) of crossing-adjacent lanelets is the crossing point, which is often on the middle of the lanelet centerlines
                % Check if one of the two vehicles has already passed the LCP (this is possible because if one vehicle's center of gravity has 
                % shortly passed the LCP, its body could still collide with another vehicle) 

                % First, find the two closest points to the crossing point on the lanelet
                distances_to_crosspoint_i = sum(([veh_info_i.lanelet_x,veh_info_i.lanelet_y]-lanelet_relationship.point).^2,2);
                [~,idx_closest_two_point_to_crosspoint_i] = mink(distances_to_crosspoint_i,2,1);
                % Next, find the two closest points to the current position on the vehicle i
                distances_to_position_i = sum(([veh_info_i.lanelet_x,veh_info_i.lanelet_y]-veh_info_i.position).^2,2);
                [~,idx_closest_two_point_to_position_i] = mink(distances_to_position_i,2,1);
                % Check wether vehicle i has passed the LCP by comparing the resulting indices 
                if min(idx_closest_two_point_to_position_i) > min(idx_closest_two_point_to_crosspoint_i)
                    % Vehicle i has passed the LCP, set the ACP to the center point of the
                    % overlapping area of their reachable sets
                    [x_centroid,y_centroid] = centroid(overlap_reachable_sets);
                    lanelet_relationship.point = [x_centroid,y_centroid];
                else
                    % If vehicle i has not passed, check if vehicle j has passed the LCP
                    distances_to_crosspoint_j = sum(([veh_info_j.lanelet_x,veh_info_j.lanelet_y]-lanelet_relationship.point).^2,2);
                    [~,idx_closest_two_point_to_crosspoint_j] = mink(distances_to_crosspoint_j,2,1);
                    distances_to_position_j = sum(([veh_info_j.lanelet_x,veh_info_j.lanelet_y]-veh_info_j.position).^2,2);
                    [~,idx_closest_two_point_to_position_j] = mink(distances_to_position_j,2,1);
                    if min(idx_closest_two_point_to_position_j) > min(idx_closest_two_point_to_crosspoint_j)
                        % Vehicle i has passed the LCP, set the ACP to the center point of the
                        % overlapping area of their reachable sets
                        [x_centroid,y_centroid] = centroid(overlap_reachable_sets);
                        lanelet_relationship.point = [x_centroid,y_centroid];
                    end
                end
            elseif lanelet_type.is_merging || (is_mixed_traffic && lanelet_type.is_left_or_right)
                % For two vehicles dirve at merging lanelets, both two collision types are possible:
                % 1. Rear-end collision: if the difference between their distances to collision point is larger than a certain velue (such as 1.5*vehicleLength)
                % 2. Side-impact collision: otherwise

                % Get arc distance to collision point 
                [arc_distance_i, ~, ~, ~, ~, ~] = get_arc_distance_to_endpoint(veh_info_i.position(1), veh_info_i.position(2), veh_info_j.lanelet_x, veh_info_j.lanelet_y);
                [arc_distance_j, ~, ~, ~, ~, ~] = get_arc_distance_to_endpoint(veh_info_j.position(1), veh_info_j.position(2), veh_info_j.lanelet_x, veh_info_j.lanelet_y); 

                safety_factor = 1;
                if abs(arc_distance_i-arc_distance_j) > safety_factor*(veh_info_i.length+veh_info_j.length)/2
                    collision_type.is_rear_end = true;
                else
                    collision_type.is_side_impact = true;
                    if obj.check_if_move_parallel(veh_info_i.position,veh_info_j.position,veh_info_i.yaw,veh_info_j.yaw,veh_info_i.length,veh_info_j.length)
                        is_move_side_by_side = true;
                    end
                end  
            else
                error('Unknown lanelet relationship.')
            end
        end

        function has_ROW = determine_who_has_ROW(obj, veh_i, veh_j, distance_to_collision_i, distance_to_collision_j, is_at_intersection, is_forking_lanelets, random_seed, scenario, iter)
        % Determine whos has the right-of-way. 
        % Vehicle enters the intersection earlier has the right-of-way. If both vheicle enter the intersection at the same time, right-of-way is given to
        % vehicle which has the right-of-way in the last time step. If they are not coupled at previous time step, vehicle who can arrive the collision point
        % earilier has the right-of-way.
        
            has_ROW = [];

            switch obj.priority_option
                case 'STAC_priority'
                    if is_at_intersection
                        if obj.time_enter_intersection(veh_i) < obj.time_enter_intersection(veh_j)
                            has_ROW = true; 
                        elseif obj.time_enter_intersection(veh_i) > obj.time_enter_intersection(veh_j)
                            has_ROW = false; 
                        else
                            % vehicle which has the right-of-way before will
                            % continually has the right-of-way
                            if obj.directed_adjacency_old(veh_i,veh_j) == 1
                                has_ROW = true; 
                            elseif obj.directed_adjacency_old(veh_j,veh_i) == 1
                                has_ROW = false;
                            else
                                has_ROW = [];
                            end
                        end
                    end

                    if isempty(has_ROW)
                        % if they are not coupled at previous time step, vehicle closer to the collision point is the leader
                        if distance_to_collision_i < distance_to_collision_j
                            has_ROW = true; 
                        else
                            has_ROW = false; 
                        end
            
                        % for forking lenelts, vehicle closer to the collision point is the follower
                        if is_forking_lanelets
                            has_ROW = ~has_ROW;
                        end
                    end
                case 'random_priority'
                    [~,~,priority_list] = random_priority().priority(scenario);
                    has_ROW = (priority_list(veh_i) <= priority_list(veh_j));
                case 'constant_priority'
                    [~,~,priority_list] = constant_priority().priority(scenario);
                    has_ROW = (priority_list(veh_i) <= priority_list(veh_j));
                case 'FCA_priority'
                    [~,~,~,priority_list] = FCA_priority().priority(scenario, iter);
                    has_ROW = (priority_list(veh_i) <= priority_list(veh_j));
                case 'coloring_priority'
                    [~,~,priority_list] = coloring_priority().priority(scenario);
                    has_ROW = (priority_list(veh_i) <= priority_list(veh_j));
                otherwise
                    warning("Priority must be one of the following: 'STAC_priority', 'random_priority', 'constant_priority'.")
            end
            
        end

        
        function obj = inherit_right_of_way(obj, predicted_lanelets)
        % If two vehicles have the same reference path, drive successively and are very close to each other,
        % the back vehicle will inherit the right-of-way of the front vehicle  
                
            coupling_weights_adjusted = obj.coupling_weights; % make a copy
            
            find_small_STAC = [obj.coupling_info.STAC] < obj.STAC_threshold; % find coupling pairs with a small STAC
        
            if ~any(find_small_STAC)
                return
            end
        
            all_coupling_pairs = [obj.coupling_info.veh_with_ROW;obj.coupling_info.veh_without_ROW];
            for iVeh = 1:obj.nVeh
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
                    find_vehs_with_ROW = [obj.coupling_info.veh_without_ROW]==veh_with_ROW;
                    find_front_vehs = all([find_vehs_with_ROW; find_small_STAC],1);
                    front_vehs_indices = find(find_front_vehs);
                    front_vehs_to_delete = [];
        
                    % only keep those front vehicles that have the same reference path as the selected vehicle
                    for j = 1:length(front_vehs_indices)
                        j_coupling = front_vehs_indices(j);
                        veh_j = obj.coupling_info(j_coupling).veh_with_ROW;
        
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
                    [~,idx_min_STAC] = min([obj.coupling_info(front_vehs_indices).STAC]);
                    i_coupling = front_vehs_indices(idx_min_STAC);
                    veh_with_ROW = obj.coupling_info(i_coupling).veh_with_ROW;
                    
                    if veh_with_ROW == iVeh
                        % in case of coupling circle
                       break
                    end
                    vehs_inherit(end+1) = veh_with_ROW;
                end
        
                if isempty(vehs_inherit)
                    continue
                end
        
                % inherit the right-of-way from the front vehicle
                % find all coupled vehicles that have side-impact collision possibility with the selected vehicle 
                vehs_with_ROW_j = find(obj.coupling_weights(:,iVeh) ~= 0);
        
                for veh_with_ROW_j = vehs_with_ROW_j(:)'
                    j_coupling = find(all(all_coupling_pairs-[veh_with_ROW_j;iVeh]==0,1));
        
                    
                    if strcmp(obj.coupling_info(j_coupling).lanelet_relationship, LaneletRelationshipType.type_3)...
                            || strcmp(obj.coupling_info(j_coupling).lanelet_relationship, LaneletRelationshipType.type_5)
                        % check if the coupled vehicle and the selected vehicle are at merging or crossing lanelets
                        find_vehs_to_inherit = obj.coupling_weights(vehs_inherit,veh_with_ROW_j)~=0;
                        find_vehs_to_inherit = vehs_inherit(find_vehs_to_inherit);
                        for veh_to_inherit = find_vehs_to_inherit(:)'
                            k_coupling = all(all_coupling_pairs-[veh_to_inherit;veh_with_ROW_j]==0,1);
                            if strcmp(obj.coupling_info(k_coupling).lanelet_relationship, LaneletRelationshipType.type_3)...
                                    || strcmp(obj.coupling_info(k_coupling).lanelet_relationship, LaneletRelationshipType.type_5)
                                % check if the coupled vehicle and the vehicle to be inherited are at merging or crossing lanelets
                                % disp(['After inheriting right-of-way from vehicle ' num2str(veh_to_inherit) ', ' num2str(iVeh) ' now has the right-of-way over ' num2str(veh_with_ROW_j) '.'])
                                % update coupling direction
                                coupling_weights_adjusted(veh_with_ROW_j,iVeh) = 0;
                                coupling_weights_adjusted(iVeh,veh_with_ROW_j) = obj.coupling_weights(veh_with_ROW_j,iVeh);
                                if veh_with_ROW_j==obj.coupling_info(j_coupling).veh_with_ROW
                                    % update coupling information
                                    [obj.coupling_info(j_coupling).veh_with_ROW, obj.coupling_info(j_coupling).veh_without_ROW] = ...
                                        swap(obj.coupling_info(j_coupling).veh_with_ROW, obj.coupling_info(j_coupling).veh_without_ROW);
                                end
                                % inherit the time entering the intersection
                                obj.time_enter_intersection(iVeh) = min(obj.time_enter_intersection(vehs_inherit));
                                % parallel driving vehicles also inherit the right-of-way
                            end
                        end
                    end
                end
                
            end        
            % update coupling weights
            obj.coupling_weights = coupling_weights_adjusted;
        end

    end



    methods (Static)
        function coupling_weight = weighting_function(STAC, sensitive_factor)
            % returns the coupling weight based on the shortest time to achieve a
            % collision (STAC)
            if nargin == 1
                sensitive_factor = 1; % default value 1
            end
            coupling_weight = exp(-sensitive_factor*STAC); % negative exponential function
        end

        function is_move_side_by_side = check_if_move_parallel(position_i,position_j,yaw_i,yaw_j,length_i,length_j)
        % Returns true if two vehicles drive parallel to each other
        % Their current occupied area are firstly approximated by straight lines. They are drive in parallel if at least one of the
        % projection of two points of one line is on the other line.
            is_move_side_by_side = false;

            % Two vehicles must be close enough to be considered to be moving in parallel 
            if norm(position_i-position_j) >= 1.2*(length_i+length_j)/2
                return
            end

            approximated_line_f = @(x0,y0,s,c,length) [x0 + length/2*c, y0 + length/2*s];
            approximated_line_r = @(x0,y0,s,c,length) [x0 - length/2*c, y0 - length/2*s];

            % approximated line of vehicle i
            s_i = sin(yaw_i); c_i = cos(yaw_i);
            point_f_i = approximated_line_f(position_i(1),position_i(2),s_i,c_i,length_i);
            point_r_i = approximated_line_r(position_i(1),position_i(2),s_i,c_i,length_i);
            s_j = sin(yaw_j); c_j = cos(yaw_j);
            point_f_j = approximated_line_f(position_j(1),position_j(2),s_j,c_j,length_j);
            point_r_j = approximated_line_r(position_j(1),position_j(2),s_j,c_j,length_j);
            
            % compact code but needs more computation
%             % the projected point is on the target line if 0<=lambda<=1
%             if ((lambda_f_i>=0 && lambda_f_i<=1) || (lambda_r_i>=0 && lambda_r_i<=1)) &&...
%                     ((lambda_f_j>=0 && lambda_f_j<=1) || (lambda_r_j>=0 && lambda_r_j<=1))
%                 is_move_parallel_tmp = true;
%             else
%                 is_move_parallel_tmp = false;
%             end

            % less compact, but computationally faster 
            % projection of two points of one line onto another line
            [~,~,~,lambda_f_i,~] = Projection2D(point_f_j(1),point_f_j(2),point_r_j(1),point_r_j(2),point_f_i(1),point_f_i(2));
            if lambda_f_i>=0 && lambda_f_i<=1
                [~,~,~,lambda_f_j,~] = Projection2D(point_f_i(1),point_f_i(2),point_r_i(1),point_r_i(2),point_f_j(1),point_f_j(2));
                if lambda_f_j>=0 && lambda_f_j<=1
                    is_move_side_by_side = true;
                else
                    [~,~,~,lambda_r_j,~] = Projection2D(point_f_i(1),point_f_i(2),point_r_i(1),point_r_i(2),point_r_j(1),point_r_j(2)); 
                    if lambda_r_j>=0 && lambda_r_j<=1
                        is_move_side_by_side = true;
                    end
                end
            else
                [~,~,~,lambda_r_i,~] = Projection2D(point_f_j(1),point_f_j(2),point_r_j(1),point_r_j(2),point_r_i(1),point_r_i(2));
                if lambda_r_i>=0 && lambda_r_i<=1
                    [~,~,~,lambda_f_j,~] = Projection2D(point_f_i(1),point_f_i(2),point_r_i(1),point_r_i(2),point_f_j(1),point_f_j(2));
                    if lambda_f_j>=0 && lambda_f_j<=1
                        is_move_side_by_side = true;
                    else
                        [~,~,~,lambda_r_j,~] = Projection2D(point_f_i(1),point_f_i(2),point_r_i(1),point_r_i(2),point_r_j(1),point_r_j(2)); 
                        if lambda_r_j>=0 && lambda_r_j<=1
                            is_move_side_by_side = true;
                        end
                    end
                end
            end
        end 
    end
end

