classdef TrafficInfo
    % TrafficInfo Traffic information, such as which vehicle is coupled
    % with which vehicle, how strong are they coupled, which vehicles are
    % at the intersection and when did they enter the intersection

    properties
        coupling_weights        % (nVeh-by-nVeh matrix) coupling weights of all coupling vehicle pair; higher value indicates stronger coupling
        directed_adjacency_old  % (nVeh-by-nVeh matrix) previous directed adjacency matrix
        coupling_info           % (struct, length equals to number of coupling pair) information of each two coupled vehicles
        vehs_at_intersection    % (vector) vehicles at intersection
        time_enter_intersection % (vector) time at which vehicle enter intersection. Value is inf if vehicle is not at intersection
        nVeh                    % total number of vehicles
    end

    properties (Constant)
        STAC_threshold = 0.6;   % vehicles are considered as very close if they can achieve a collision in less than this time
        sensitive_factor = 1;   % sensitive factor used to calculate the coupling weights. The bigger, the more sensitive to the STAC. Default value 1.
    end

    methods
        function obj = TrafficInfo(scenario, iter)
            obj.nVeh = scenario.nVeh;
            
            % initialize
            obj.directed_adjacency_old = zeros(obj.nVeh,obj.nVeh);
            obj.coupling_weights = zeros(obj.nVeh,obj.nVeh); 
            % ROW stands for right-of-way
            obj.coupling_info = struct('veh_with_ROW',[],'veh_without_ROW',[],'collision_type',[],...
                'lanelet_relationship',[],'STAC',[],'is_at_intersection',[],'is_drive_parallel',[],'is_ignored',[]); 

            % estimate traffic information
            obj = obj.estimate_traffic_info(scenario,iter);
        end
        
        function obj = estimate_traffic_info(obj, scenario, iter)
            if scenario.k>1
                for i = 1:length([scenario.coupling_info.veh_with_ROW])
                    veh_with_ROW = scenario.coupling_info(i).veh_with_ROW;
                    veh_without_ROW = scenario.coupling_info(i).veh_without_ROW;
                    obj.directed_adjacency_old(veh_with_ROW,veh_without_ROW) = 1;
                end
            end
        
            obj = obj.update_vehs_at_intersection(scenario,iter.x0);

            % get bounding box of reachable sets in the last prediction horizon
            bound_boxes_x = zeros(obj.nVeh,2); bound_boxes_y = zeros(obj.nVeh,2);
            for iVeh = 1:obj.nVeh
                [bound_boxes_x(iVeh,:),bound_boxes_y(iVeh,:)] = boundingbox(iter.reachable_sets{iVeh,end});
            end

            for veh_i = 1:(obj.nVeh-1)
                x_i = bound_boxes_x(veh_i,:);
                y_i = bound_boxes_y(veh_i,:);
                
                for veh_j = (veh_i+1):obj.nVeh  
                    % check whether two vehicles' reachable sets at the last prediction horizon overlap
                    x_j = bound_boxes_x(veh_j,:);
                    y_j = bound_boxes_y(veh_j,:);
                    % use rectangles to approximate their reachable sets for a quick check
                    if x_i(1)>x_j(2) || y_i(1)>y_j(2) || x_i(2)<x_j(1) || y_i(2)<y_j(1)
                        % reachable sets are not overlapping
                        continue
                    end
                    overlap_reachable_sets = intersect(iter.reachable_sets{veh_i,end}, iter.reachable_sets{veh_j,end});
                    area_overlap = area(overlap_reachable_sets);
                    if area_overlap > 1e-3 % a small threshold to tolerate measurement error of lenelet boundary
                        % the selected two vehicles are considered as coupled if their reachable sets overlap
                        obj = obj.get_coupling_info(scenario,iter,veh_i,veh_j,overlap_reachable_sets);
                    end  
                end
            end
        
            % If two vehicles drive successively and are very close to each other,
            % the back vehicle will inherit the right-of-way of the front vehicle 
            obj = obj.inherit_right_of_way(iter.predicted_lanelets);
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
            obj.vehs_at_intersection = find(distances_to_center < 1.1);
        
            new_veh_at_intersection = setdiff(obj.vehs_at_intersection, scenario.last_vehs_at_intersection);
        
            veh_leave_intersection = setdiff(scenario.last_vehs_at_intersection, obj.vehs_at_intersection);
            obj.time_enter_intersection(new_veh_at_intersection) = scenario.k;
            obj.time_enter_intersection(veh_leave_intersection) = inf; % set to inf if vehicle leaves the intersection
        end


        function obj = get_coupling_info(obj,scenario,iter,veh_i,veh_j,overlap_reachable_sets)
            % This function estimate the information of each two coupled vehicles
            stop_flag = false;
            count = length([obj.coupling_info.veh_with_ROW]) + 1; % for the next coupling infomation 

            veh_info_i = struct('position',[],'yaw',[],'trim',[],'predicted_lanelet',[],'length',[],...
                'lanelet_x',[],'lanelet_y',[]);
            veh_info_i.position = [iter.x0(veh_i,indices().x), iter.x0(veh_i,indices().y)];
            veh_info_i.yaw = iter.x0(veh_i,indices().heading);
            veh_info_i.trim = iter.trim_indices(veh_i);
            veh_info_i.length = scenario.vehicles(veh_i).Length;
            predicted_lanelets_i = iter.predicted_lanelets{veh_i};

            veh_info_j = struct('position',[],'yaw',[],'trim',[],'predicted_lanelet',[],'length',[],...
                'lanelet_x',[],'lanelet_y',[]);
            veh_info_j.position = [iter.x0(veh_j,indices().x), iter.x0(veh_j,indices().y)];
            veh_info_j.yaw = iter.x0(veh_j,indices().heading);
            veh_info_j.trim = iter.trim_indices(veh_j);
            veh_info_j.length = scenario.vehicles(veh_j).Length;
            predicted_lanelets_j = iter.predicted_lanelets{veh_j};

            lanelet = struct('x_i',[],'y_i',[],'x_j',[],'y_j',[]);

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

                    [lanelet_type,collision_type,is_continue,lanelet_relationship,is_find_lanelet_relationship] = ...
                        obj.get_collision_and_lanelet_type(veh_info_i,veh_info_j,is_last_lan_pair,scenario.lanelet_relationships,overlap_reachable_sets, scenario.options.is_mixed_traffic);
    
                    % vehicle in Expert-Mode has only a single predicted lanelet
                    if length(predicted_lanelets_i) > 1 && length(predicted_lanelets_j) > 1
                        if predicted_lanelets_i(2) == predicted_lanelets_j(1) || predicted_lanelets_i(1) == predicted_lanelets_j(2)
                            disp('')
                        end
                    end

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

                    % estimate the STAC and coupling weight
                    if collision_type.is_rear_end
                        % If the same lanelet or successive (relationship type 1) lanelets or adjacent left/right lanelets (relationship type 2) -> rear-end collision
                        obj.coupling_info(count).collision_type = CollisionType.type_1;
                        % only side-impact collision is possible for vehicles driving in parallel
                        obj.coupling_info(count).is_drive_parallel = false; 
    
                        % the same lanelet ot successive lanelet or forking lanelets
                        collision_point = lanelet_relationship.point;
                        distance_to_collision_i = norm(veh_info_i.position-collision_point,2);
                        distance_to_collision_j = norm(veh_info_j.position-collision_point,2);
                        
                        % determine who is the leader 
                        has_ROW = obj.determine_who_has_ROW(veh_i, veh_j, distance_to_collision_i, distance_to_collision_j, obj.coupling_info(count).is_at_intersection, lanelet_type.is_forking);
                        
                        if has_ROW                        
                            trim_with_ROW = veh_info_i.trim; trim_without_ROW = veh_info_j.trim;
                        else
                            trim_with_ROW = veh_info_j.trim; trim_without_ROW = veh_info_i.trim;
                        end
    
                        % calculate the shortest time to achieve a collision
                        [STAC, waiting_time, ~, ~] = get_the_shortest_time_to_catch(scenario.mpa, trim_with_ROW, trim_without_ROW, distance_two_vehs, scenario.dt);
                        STAC_adapted = STAC + waiting_time;
    
                    elseif collision_type.is_side_impact
                        obj.coupling_info(count).collision_type = CollisionType.type_2; % side-impact collision
    
                        % For side-impact collision, we calculate the adapted STAC, which is the sum of the actual STAC and a waiting time. 
                        % The calculation procedure is the following: Firstly, the actual STAC is the shortest time needed to achieve a 
                        % collision at the point of intersection. This is calculated by letting both vehicles take a full acceleration to 
                        % arrive at the point of intersection. The first arrived vehicle has to stop at the point of intersection and wait 
                        % for the second arrived vehicle, where the waiting time is the second part of the adapted STAC.
                        collision_point = lanelet_relationship.point;
                        if ~is_find_lanelet_relationship || lanelet_type.is_same || lanelet_type.is_successive
                            distance_to_collision_i = norm(veh_info_i.position-collision_point);
                            distance_to_collision_j = norm(veh_info_j.position-collision_point);
                        else
                            % from a curve to calculate the arc diatance between vehicle's current position and the collision point, 
                            % which starts from the starting point of the lanelet and ends at the collision point 
                            if lanelet_type.is_merging || lanelet_type.is_adjacent_same
                                % collision point of the merging lanelets is both lanelets' endpoint, thus the target curve is the whole lanelet
                                curve_x_i = veh_info_i.lanelet_x; curve_y_i = veh_info_i.lanelet_y;
                                curve_x_j = veh_info_j.lanelet_x; curve_y_j = veh_info_j.lanelet_y;
                            elseif lanelet_type.is_intersecting 
                                % Collision point of the intersecting lanelets is the crosspoint, which could be in the middle of the lanelets
                                % First, find the two closest points to the crosspoint on the lanelet
                                squared_distances_to_crosspoint_i = sum(([veh_info_i.lanelet_x,veh_info_i.lanelet_y]-collision_point).^2,2);
                                [~,idx_closest_two_point_i] = mink(squared_distances_to_crosspoint_i,2,1);
                                squared_distances_to_crosspoint_j = sum(([veh_info_j.lanelet_x,veh_info_j.lanelet_y]-collision_point).^2,2);
                                [~,idx_closest_two_point_j] = mink(squared_distances_to_crosspoint_j,2,1);
                                % the endpoint of the curve is the collision point and the adjacent left point is the one among the closest two points with a smaller index  
                                curve_x_i = [veh_info_i.lanelet_x(1:min(idx_closest_two_point_i));collision_point(1)]; curve_y_i = [veh_info_i.lanelet_y(1:min(idx_closest_two_point_i));collision_point(2)];
                                curve_x_j = [veh_info_j.lanelet_x(1:min(idx_closest_two_point_j));collision_point(1)]; curve_y_j = [veh_info_j.lanelet_y(1:min(idx_closest_two_point_j));collision_point(2)];
                            end
    
                            % calculate the arc length from the vehicle's current position to the collision point
                            [distance_to_collision_i, ~, ~, ~, ~, ~] = get_arc_distance_to_endpoint(veh_info_i.position(1), veh_info_i.position(2), curve_x_i, curve_y_i);
                            [distance_to_collision_j, ~, ~, ~, ~, ~] = get_arc_distance_to_endpoint(veh_info_j.position(1), veh_info_j.position(2), curve_x_j, curve_y_j);
                        end
    
                        % determine who is the leader 
                        has_ROW = obj.determine_who_has_ROW(veh_i, veh_j, distance_to_collision_i, distance_to_collision_j, obj.coupling_info(count).is_at_intersection, lanelet_type.is_forking);
    
                        if lanelet_type.is_same || lanelet_type.is_successive
                            % two vehicles drive in parallel if they drive at the same lanelet or successive lanelets and have side-impact collision possibility 
                            obj.coupling_info(count).is_drive_parallel = true; 
                            % calculate the STAC simply by dividing their distance minus their width by half the maximum speed
                            % todo: find a better heuristic to calculate the STAC for vehicles driving in parallel
                            distance_to_collision = distance_two_vehs-(scenario.vehicles(veh_i).Width+scenario.vehicles(veh_j).Width)/2;
                            assert(distance_to_collision>0)
                            STAC = distance_to_collision/max([scenario.mpa.trims.speed]./2);
                            STAC_adapted = STAC;
                        else
                            obj.coupling_info(count).is_drive_parallel = false; 
                            time_to_collision_point_i = get_the_shortest_time_to_arrive(scenario.mpa,veh_info_i.trim,distance_to_collision_i,scenario.dt);
                            time_to_collision_point_j = get_the_shortest_time_to_arrive(scenario.mpa,veh_info_j.trim,distance_to_collision_j,scenario.dt);
                            % the first arrived vehicle will wait for the second arrived vehicle to "achieve" a collision.
                            % Here we ignore the fact that the first arrived vehicle should also decelerate to stop at the
                            % point of intersection, which could be added to todo list to improve the exactness.
                            waiting_time = abs(time_to_collision_point_i-time_to_collision_point_j);
                            % actual STAC
                            STAC = max(time_to_collision_point_i,time_to_collision_point_j);
                            % adapted STAC
                            STAC_adapted = STAC + waiting_time;   
                        end

                    end
    
                    % store coupling information
                    obj.coupling_info(count).STAC = STAC;
                    obj.coupling_info(count).lanelet_relationship = lanelet_relationship.type;
                    obj.coupling_info(count).is_ignored = false; % whether the coupling is ignored  
                    if has_ROW
                        % vehicle_i has the right-of-way
                        obj.coupling_weights(veh_i,veh_j) = obj.weighting_function(STAC_adapted, obj.sensitive_factor); 
                        obj.coupling_info(count).veh_with_ROW = veh_i;  
                        obj.coupling_info(count).veh_without_ROW = veh_j; 
                    else
                        % vehicle_j has the right-of-way
                        obj.coupling_weights(veh_j,veh_i) = obj.weighting_function(STAC_adapted, obj.sensitive_factor); 
                        obj.coupling_info(count).veh_with_ROW = veh_j;
                        obj.coupling_info(count).veh_without_ROW = veh_i;
                    end
                    stop_flag = true;
                    break  
                end
                if stop_flag
                    break
                end
            end
        end


        function [lanelet_type,collision_type,is_continue,lanelet_relationship,is_find_lanelet_relationship] = ...
                get_collision_and_lanelet_type(obj,veh_info_i,veh_info_j,is_last_lan_pair,lanelet_relationships,overlap_reachable_sets, is_mixed_traffic)
        
            % initialize
            lanelet_type = struct('is_same',false,'is_adjacent_same',false,'is_forking',false,'is_merging',false,'is_intersecting',false,'is_successive',false);
            collision_type = struct('is_rear_end',false,'is_side_impact',false);
            is_continue = false;
            lanelet_relationship = struct('type',[],'point',[]); 
            is_find_lanelet_relationship = false;
        
            % get lanelet relationship
            if veh_info_i.predicted_lanelet == veh_info_j.predicted_lanelet
                lanelet_type.is_same = true; % the same lanelet
                % create information of lanelet relationship
                lanelet_relationship.type = LaneletRelationshipType.type_6; % same lanelet
                lanelet_relationship.point = [veh_info_i.lanelet_x(end),veh_info_i.lanelet_y(end)]; % store the endpoint of the lanelet
                
%                 % Vehicles drive in parallel if the distance of the
%                 % projections of their center points to lanelet center line
%                 % is smaller than the mean length of the two vehicles
%                 [~, ~, x_projected_i, y_projected_i, ~, ~] = get_arc_distance_to_endpoint(veh_info_i.position(1), veh_info_i.position(2), veh_info_i.lanelet_x, veh_info_i.lanelet_y);
%                 [~, ~, x_projected_j, y_projected_j, ~, ~] = get_arc_distance_to_endpoint(veh_info_j.position(1), veh_info_j.position(2), veh_info_j.lanelet_x, veh_info_j.lanelet_y);
%                 if norm([x_projected_i, y_projected_i]-[x_projected_j, y_projected_j]) <= vehs_mean_length
%                     collision_type.is_side_impact = true;
%                     lanelet_relationship.point = (veh_info_i.position+veh_info_j.position)./2; % middle point as the collision point
%                 else
%                     collision_type.is_rear_end = true;
%                     lanelet_relationship.point = [veh_info_i.lanelet_x(end),veh_info_i.lanelet_y(end)]; % store the endpoint of the lanelet
%                 end
            else
                % find if there exists lanelet pair that has a certain relationship in the struct array `lanelet_relationships`
                % NOTE that only adjacent lanelet pairs with certain relationships will be stored in `lanelet_relationships`
                idx_lanelet_pair = RoadData().find_idx_lanelet_pair(veh_info_i.predicted_lanelet,veh_info_j.predicted_lanelet,lanelet_relationships);
                if ~isempty(idx_lanelet_pair)
                    is_find_lanelet_relationship = true;
                    lanelet_relationship.type = lanelet_relationships(idx_lanelet_pair).type;
                    lanelet_relationship.point = lanelet_relationships(idx_lanelet_pair).point;
                else
                    if is_last_lan_pair
                        % If no lanelet relationship is found until the last predicted lanelet pair
                        % 1. Set the center point of the overlapping area as collision point
                        % 2. Set lanelet relationship to be intersecting
                        % 3. Set collision type to be side-impact collision (normally lanelet relationship will always be found for rear-end collision)
        %                 disp(['No lanelet relationship can be found for coupled vehicle ' num2str(veh_i) ' and ' num2str(veh_j) '.' newline...
        %                     'Centroid of the overlapping area of their reachable sets will be used as collision point.'])        
                        lanelet_relationship.type = LaneletRelationshipType.type_5; % intersecting lanelets
                        [x_centroid,y_centroid] = centroid(overlap_reachable_sets);
                        lanelet_relationship.point = [x_centroid,y_centroid];
                    else
                        % jump to the next predicted lanelet
                        is_continue = true;
                        return
                    end
                end
            end

            % get lanelet_type based on lanelet_relationship
            lanelet_type.is_successive = strcmp(lanelet_relationship.type, LaneletRelationshipType.type_1);
            lanelet_type.is_adjacent_same = strcmp(lanelet_relationship.type, LaneletRelationshipType.type_2);
            lanelet_type.is_merging = strcmp(lanelet_relationship.type, LaneletRelationshipType.type_3);
            lanelet_type.is_forking = strcmp(lanelet_relationship.type, LaneletRelationshipType.type_4);
            lanelet_type.is_intersecting = strcmp(lanelet_relationship.type, LaneletRelationshipType.type_5);
            lanelet_type.is_same = strcmp(lanelet_relationship.type, LaneletRelationshipType.type_6);

            % get collision_type type based on lanelet_type
            if lanelet_type.is_same || lanelet_type.is_successive
                % For two vehicles dirve at the same, successive or forking lanelets, both two collision types are possible: 
                % 1. Side-impact collision: if their distance is less than the mean length of the two vehicles
                % 2. Rear-end collision: otherwise
%                         if norm(veh_info_i.position-veh_info_j.position) < (veh_info_i.length+veh_info_j.length)/2
                if obj.check_if_drive_parallel(veh_info_i.position,veh_info_j.position,veh_info_i.yaw,veh_info_j.yaw,veh_info_i.length,veh_info_j.length)
                    collision_type.is_side_impact = true;
                else
                    collision_type.is_rear_end = true;
                end

            elseif lanelet_type.is_forking
                % Only rear-end collision is possible at forking lanelets
                collision_type.is_rear_end = true;
            elseif lanelet_type.is_intersecting
                % Only side-impact collision is possible at intersection lanelets
                collision_type.is_side_impact = true;
            elseif lanelet_type.is_merging || (is_mixed_traffic && lanelet_type.is_adjacent_same)
                % For two vehicles dirve at merging lanelets, both two collision types are possible:
                % 1. Rear-end collision: if the difference between their distances to collision point is larger than a certain velue (such as 1.5*vehicleLength)
                % 2. Side-impact collision: otherwise

                % get arc distance to collision point 
                [arc_distance_i, ~, ~, ~, ~, ~] = get_arc_distance_to_endpoint(veh_info_i.position(1), veh_info_i.position(2), veh_info_j.lanelet_x, veh_info_j.lanelet_y);
                [arc_distance_j, ~, ~, ~, ~, ~] = get_arc_distance_to_endpoint(veh_info_j.position(1), veh_info_j.position(2), veh_info_j.lanelet_x, veh_info_j.lanelet_y); 

                safety_factor = 1.5;
                if abs(arc_distance_i-arc_distance_j) > safety_factor*(veh_info_i.length+veh_info_j.length)/2
%                 if norm([x_projected_i,y_projected_i]-veh_info_j.position) > wheelbase_mean && norm([x_projected_j,y_projected_j]-veh_info_i.position) > wheelbase_mean
                    collision_type.is_rear_end = true;
                else
                    collision_type.is_side_impact = true;
                end  
            else
                error('Unknown lanelet relationship.')
            end
        
        end


        function has_ROW = determine_who_has_ROW(obj, veh_i, veh_j, distance_to_collision_i, distance_to_collision_j, is_at_intersection, is_forking_lanelets)
        % Determine whos has the right-of-way. 
        % Vehicle enters the intersection earlier has the right-of-way. If both
        % vheicle enter the intersection at the same time, right-of-way is given to
        % vehicle which has the right-of-way in the last time step. If they are not
        % coupled at previous time step, vehicle who can arrive the collision point
        % earilier has the right-of-way.
        
            has_ROW = [];
            
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
        
                % inherit the time entering the intersection
                obj.time_enter_intersection(iVeh) = min(obj.time_enter_intersection(vehs_inherit));
        
                % inherit the right-of-way from the front vehicle
                % find all coupled vehicles that have side-impact collision possibility with the selected vehicle 
                vehs_with_ROW_j = find(obj.coupling_weights(:,iVeh) ~= 0);
        
                for veh_with_ROW_j = vehs_with_ROW_j(:)'
                    j_coupling = find(all(all_coupling_pairs-[veh_with_ROW_j;iVeh]==0,1));
        
                    % check if the coupled vehicle has side-impact collision
                    % possibility both with the selected vehicle and the vehicle to be inherited 
                    if strcmp(obj.coupling_info(j_coupling).collision_type, CollisionType.type_2)
                        % side-impact collision with the selected vehicle
                        find_vehs_to_inherit = obj.coupling_weights(vehs_inherit,veh_with_ROW_j)~=0;
                        find_vehs_to_inherit = vehs_inherit(find_vehs_to_inherit);
                        for veh_to_inherit = find_vehs_to_inherit(:)'
                            k_coupling = all(all_coupling_pairs-[veh_to_inherit;veh_with_ROW_j]==0,1);
                            if strcmp(obj.coupling_info(k_coupling).collision_type, CollisionType.type_2)
                                % side-impact collision with the vehicle to be inherited 
                                disp(['After inheriting right-of-way from vehicle ' num2str(veh_to_inherit) ', ' num2str(iVeh) ' now has the right-of-way over ' num2str(veh_with_ROW_j) '.'])
                                % update coupling direction
                                coupling_weights_adjusted(veh_with_ROW_j,iVeh) = 0;
                                coupling_weights_adjusted(iVeh,veh_with_ROW_j) = obj.coupling_weights(veh_with_ROW_j,iVeh);
                                
                                % update coupling information
                                [obj.coupling_info(j_coupling).veh_with_ROW, obj.coupling_info(j_coupling).veh_without_ROW] = ...
                                    swap(obj.coupling_info(j_coupling).veh_with_ROW, obj.coupling_info(j_coupling).veh_without_ROW);
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

        function is_drive_parallel = check_if_drive_parallel(position_i,position_j,yaw_i,yaw_j,length_i,length_j)
        % Returns true if two vehicles drive parallel to each other
        % Their current occupied area are firstly approximated by straight
        % lines. They are drive in parallel if at least one of the
        % projection of two points of one line is on the other line.
            approximated_line_f = @(x0,y0,s,c,length) [x0 + length/2*c, y0 + length/2*s];
            approximated_line_r = @(x0,y0,s,c,length) [x0 - length/2*c, y0 - length/2*s];

            % approximated line of vehicle i
            s_i = sin(yaw_i); c_i = cos(yaw_i);
            point_f_i = approximated_line_f(position_i(1),position_i(2),s_i,c_i,length_i);
            point_r_i = approximated_line_r(position_i(1),position_i(2),s_i,c_i,length_i);
            s_j = sin(yaw_j); c_j = cos(yaw_j);
            point_f_j = approximated_line_f(position_j(1),position_j(2),s_j,c_j,length_j);
            point_r_j = approximated_line_r(position_j(1),position_j(2),s_j,c_j,length_j);

            % projection of two points of one line onto another line
            [~,~,~,lambda_f_i,~] = Projection2D(point_f_j(1),point_f_j(2),point_r_j(1),point_r_j(2),point_f_i(1),point_f_i(2));
            [~,~,~,lambda_r_i,~] = Projection2D(point_f_j(1),point_f_j(2),point_r_j(1),point_r_j(2),point_r_i(1),point_r_i(2));
            [~,~,~,lambda_f_j,~] = Projection2D(point_f_i(1),point_f_i(2),point_r_i(1),point_r_i(2),point_f_j(1),point_f_j(2));
            [~,~,~,lambda_r_j,~] = Projection2D(point_f_i(1),point_f_i(2),point_r_i(1),point_r_i(2),point_r_j(1),point_r_j(2)); 
            % the projected point is on the target line if 0<=lambda<=1
            if (lambda_f_i>=0 && lambda_f_i<=1) || (lambda_r_i>=0 && lambda_r_i<=1) ||...
                    (lambda_f_j>=0 && lambda_f_j<=1) || (lambda_r_j>=0 && lambda_r_j<=1)
                is_drive_parallel = true;
            else
                is_drive_parallel = false;
            end
        end 
    end
end

