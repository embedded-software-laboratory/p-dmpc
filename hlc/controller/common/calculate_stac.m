function [stac, distance_to_collision_i, distance_to_collision_j, collision_type, lanelet_relationship, is_move_side_by_side] = calculate_stac(veh_i, veh_j, dt_seconds, scenario, mpa, iter)
    % CALCULATE_STAC        calculate the shortest time to achieve collision.
    %                       As co-product: save distance to collision point,
    %                       collision type and lanelet relationship type

    waiting_time_factor = 0.25;

    [collision_type, lanelet_relationship] = deal([]);
    [stac, distance_to_collision_i, distance_to_collision_j] = deal(0);

    is_move_side_by_side = false;

    overlap_reachable_sets = intersect(iter.reachable_sets{veh_i, end}, iter.reachable_sets{veh_j, end});
    lanelet_relationship_point_reachable_sets = centroid(overlap_reachable_sets);

    info_i = get_vehicle_info(veh_i, iter);
    info_j = get_vehicle_info(veh_j, iter);

    is_last_pair = false;

    for lanelet_i = info_i.predicted_lanelets

        for lanelet_j = info_j.predicted_lanelets

            % center line of the lanelets
            lanelet_i_x = scenario.lanelets{lanelet_i}(:, LaneletInfo.cx);
            lanelet_i_y = scenario.lanelets{lanelet_i}(:, LaneletInfo.cy);
            lanelet_j_x = scenario.lanelets{lanelet_j}(:, LaneletInfo.cx);
            lanelet_j_y = scenario.lanelets{lanelet_j}(:, LaneletInfo.cy);

            lanelet_pair = [lanelet_i, lanelet_j];
            lanelet_pair_sorted = sort(lanelet_pair);

            if lanelet_pair == [info_i.predicted_lanelets(end), info_j.predicted_lanelets(end)]
                is_last_pair = true;
            end

            if lanelet_i == lanelet_j
                % lanelets are the same
                lanelet_relationship = LaneletRelationshipType.same;
                lanelet_relationship_point = [lanelet_i_x(end), lanelet_i_y(end)];
            else
                % Find if there exists lanelet pair that has a certain relationship in the struct array `lanelet_relationships`
                lanelet_relationship = scenario.lanelet_relationships{lanelet_pair_sorted(1), lanelet_pair_sorted(2)};

                if isempty(lanelet_relationship) && is_last_pair
                    % If no lanelet relationship is found until the last predicted lanelet pair
                    % 1. Set the center point of the overlapping area as the assumed collision point
                    % 2. Set lanelet relationship to crossing-adjacent
                    % 3. Set collision type to side-impact collision
                    % (lanelet relationship could not found when centerlines of lanelets at intersection does not intersect, but their boundaries intersect)
                    lanelet_relationship = LaneletRelationshipType.crossing;
                    lanelet_relationship_point = lanelet_relationship_point_reachable_sets;
                elseif isempty(lanelet_relationship)
                    % Jump to the next predicted lanelet if it is not the last pair to check
                    continue
                else
                    lanelet_relationship_point = lanelet_relationship.point;
                    lanelet_relationship = lanelet_relationship.type;
                end

            end

            switch lanelet_relationship
                case LaneletRelationshipType.crossing
                    % Only side-impact collision is possible at corssing-adjacent lanelets
                    collision_type = CollisionType.from_side;
                    % The lanelet critical point (LCP) of crossing-adjacent lanelets is the crossing point, which is often on the middle of the lanelet centerlines
                    % Check if one of the two vehicles has already passed the LCP (this is possible because if one vehicle's center of gravity has
                    % shortly passed the LCP, its body could still collide with another vehicle)

                    % First, find the two closest points to the crossing point on the lanelet
                    distances_to_crosspoint_i = sum(([lanelet_i_x, lanelet_i_y] - lanelet_relationship_point).^2, 2);
                    [~, idx_closest_two_point_to_crosspoint_i] = mink(distances_to_crosspoint_i, 2, 1);
                    % Next, find the two closest points to the current position on the vehicle i
                    distances_to_position_i = sum(([lanelet_i_x, lanelet_i_y] - info_i.position).^2, 2);
                    [~, idx_closest_two_point_to_position_i] = mink(distances_to_position_i, 2, 1);
                    % Check wether vehicle i has passed the LCP by comparing the resulting indices
                    if min(idx_closest_two_point_to_position_i) > min(idx_closest_two_point_to_crosspoint_i)
                        % Vehicle i has passed the LCP, set the assumend collision point (ACP) to the center point of the
                        % overlapping area of their reachable sets
                        lanelet_relationship_point = lanelet_relationship_point_reachable_sets;
                    else
                        % If vehicle i has not passed, check if vehicle j has passed the LCP
                        distances_to_crosspoint_j = sum(([lanelet_j_x, lanelet_j_y] - lanelet_relationship_point).^2, 2);
                        [~, idx_closest_two_point_to_crosspoint_j] = mink(distances_to_crosspoint_j, 2, 1);
                        distances_to_position_j = sum(([lanelet_j_x, lanelet_j_y] - info_j.position).^2, 2);
                        [~, idx_closest_two_point_to_position_j] = mink(distances_to_position_j, 2, 1);

                        if min(idx_closest_two_point_to_position_j) > min(idx_closest_two_point_to_crosspoint_j)
                            % Vehicle i has passed the LCP, set the ACP to the center point of the
                            % overlapping area of their reachable sets
                            lanelet_relationship_point = lanelet_relationship_point_reachable_sets;
                        end

                    end

                case LaneletRelationshipType.merging
                    % For two vehicles dirve at merging lanelets, both two collision types are possible:
                    % 1. Rear-end collision: if the difference between their distances to collision point is larger than a certain velue (such as 1.5*vehicleLength)
                    % 2. Side-impact collision: otherwise

                    % Get arc distance to collision point
                    [arc_distance_i, ~, ~, ~, ~, ~] = get_arc_distance_to_endpoint(info_i.position(1), info_i.position(2), lanelet_i_x, lanelet_i_y);
                    [arc_distance_j, ~, ~, ~, ~, ~] = get_arc_distance_to_endpoint(info_j.position(1), info_j.position(2), lanelet_j_x, lanelet_j_y);

                    safety_factor = 1;

                    if abs(arc_distance_i - arc_distance_j) > safety_factor * (info_i.length + info_j.length) / 2
                        collision_type = CollisionType.from_rear;
                    else
                        collision_type = CollisionType.from_side;

                        if is_vehicles_move_parallel(info_i.position, info_j.position, info_i.yaw, info_j.yaw, info_i.length, info_j.length)
                            is_move_side_by_side = true;
                        end

                    end

                otherwise % same, side, longitudinal, forking
                    % For two vehicles at the same, longitudinal-, left/right-, or forking-adjacent lanelets, both two collision types are possible:
                    % 1. Side-impact collision: if they move in parallel
                    % 2. Rear-end collision: otherwise
                    if is_vehicles_move_parallel(info_i.position, info_j.position, info_i.yaw, info_j.yaw, info_i.length, info_j.length)
                        collision_type = CollisionType.from_side;
                        is_move_side_by_side = true;
                    else
                        collision_type = CollisionType.from_rear;
                    end

            end

            % current distance between two vehicles
            distance_two_vehs = norm(info_i.position - info_j.position, 2);
            assumed_collision_point = lanelet_relationship_point;

            distance_to_collision_i = norm(info_i.position - assumed_collision_point);
            distance_to_collision_j = norm(info_j.position - assumed_collision_point);

            switch collision_type
                case CollisionType.from_rear
                    % If two vehicles have a rear collision, their STAC
                    % is the time to catch.
                    % The leader is the vehicle closer to the collision point
                    is_vehicle_i_leader = (distance_to_collision_i < distance_to_collision_j);
                    % Except in forking lanelets, where the collision point is
                    % behind vehicles
                    if (lanelet_relationship == LaneletRelationshipType.forking)
                        is_vehicle_i_leader = ~is_vehicle_i_leader;
                    end

                    if is_vehicle_i_leader
                        trim_leader = info_i.trim;
                        trim_follower = info_j.trim;
                    else
                        trim_leader = info_j.trim;
                        trim_follower = info_i.trim;
                    end

                    [stac, waiting_time, ~, ~] = get_the_shortest_time_to_catch( ...
                        mpa, ...
                        trim_leader, ...
                        trim_follower, ...
                        distance_two_vehs, ...
                        dt_seconds ...
                    );
                case CollisionType.from_side
                    % If two vehicles has a side-impact collision possibility, check if they move in parallel
                    if is_move_side_by_side
                        stac = distance_two_vehs / 2 / max([mpa.trims.speed]) * 2;
                        waiting_time = 0;
                    else
                        time_to_collision_point_i = get_the_shortest_time_to_arrive(mpa, info_i.trim, distance_to_collision_i, dt_seconds);
                        time_to_collision_point_j = get_the_shortest_time_to_arrive(mpa, info_j.trim, distance_to_collision_j, dt_seconds);
                        % determine which vehicle has the ROW
                        % trick: a shorter time to collision point corresponds to a shorter distance to collision point, thus no code adaption is need

                        stac = max(time_to_collision_point_i, time_to_collision_point_j);
                        waiting_time = abs(time_to_collision_point_i - time_to_collision_point_j);
                    end

                otherwise
                    error('Unknown collision type');
            end

            stac = stac + waiting_time_factor * waiting_time;
            return;
        end

    end

end

%% auxilary functions

function info = get_vehicle_info(id, iter)
    info = struct('position', [], 'yaw', [], 'trim', [], 'predicted_lanelets', [], 'length', []);
    info.position = [iter.x0(id, indices().x), iter.x0(id, indices().y)];
    info.yaw = iter.x0(id, indices().heading);
    info.trim = iter.trim_indices(id);
    info.length = iter.vehicles(id).Length;
    info.predicted_lanelets = iter.predicted_lanelets{id};
end

function is_move_side_by_side = is_vehicles_move_parallel(position_i, position_j, yaw_i, yaw_j, length_i, length_j)
    % Returns true if two vehicles drive parallel to each other
    % Their current occupied area are firstly approximated by straight lines. They are drive in parallel if at least one of the
    % projection of two points of one line is on the other line.
    is_move_side_by_side = false;

    % Two vehicles must be close enough to be considered to be moving in parallel
    if norm(position_i - position_j) >= 1.2 * (length_i + length_j) / 2
        return
    end

    approximated_line_f = @(x0, y0, s, c, length) [x0 + length / 2 * c, y0 + length / 2 * s];
    approximated_line_r = @(x0, y0, s, c, length) [x0 - length / 2 * c, y0 - length / 2 * s];

    % approximated line of vehicle i
    s_i = sin(yaw_i); c_i = cos(yaw_i);
    point_f_i = approximated_line_f(position_i(1), position_i(2), s_i, c_i, length_i);
    point_r_i = approximated_line_r(position_i(1), position_i(2), s_i, c_i, length_i);
    s_j = sin(yaw_j); c_j = cos(yaw_j);
    point_f_j = approximated_line_f(position_j(1), position_j(2), s_j, c_j, length_j);
    point_r_j = approximated_line_r(position_j(1), position_j(2), s_j, c_j, length_j);

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
    [~, ~, ~, lambda_f_i, ~] = projection_2d(point_f_j(1), point_f_j(2), point_r_j(1), point_r_j(2), point_f_i(1), point_f_i(2));

    if lambda_f_i >= 0 && lambda_f_i <= 1
        [~, ~, ~, lambda_f_j, ~] = projection_2d(point_f_i(1), point_f_i(2), point_r_i(1), point_r_i(2), point_f_j(1), point_f_j(2));

        if lambda_f_j >= 0 && lambda_f_j <= 1
            is_move_side_by_side = true;
        else
            [~, ~, ~, lambda_r_j, ~] = projection_2d(point_f_i(1), point_f_i(2), point_r_i(1), point_r_i(2), point_r_j(1), point_r_j(2));

            if lambda_r_j >= 0 && lambda_r_j <= 1
                is_move_side_by_side = true;
            end

        end

    else
        [~, ~, ~, lambda_r_i, ~] = projection_2d(point_f_j(1), point_f_j(2), point_r_j(1), point_r_j(2), point_r_i(1), point_r_i(2));

        if lambda_r_i >= 0 && lambda_r_i <= 1
            [~, ~, ~, lambda_f_j, ~] = projection_2d(point_f_i(1), point_f_i(2), point_r_i(1), point_r_i(2), point_f_j(1), point_f_j(2));

            if lambda_f_j >= 0 && lambda_f_j <= 1
                is_move_side_by_side = true;
            else
                [~, ~, ~, lambda_r_j, ~] = projection_2d(point_f_i(1), point_f_i(2), point_r_i(1), point_r_i(2), point_r_j(1), point_r_j(2));

                if lambda_r_j >= 0 && lambda_r_j <= 1
                    is_move_side_by_side = true;
                end

            end

        end

    end

end

function [time_to_catch, waiting_time, distance_traveled_leader_total, distance_traveled_follower_total] = get_the_shortest_time_to_catch(mpa, trim_leader, trim_follower, distance, time_step)
    % Calculatet the shortest time to achieve a collision by
    % letting the leader take an emergency braking and the follower
    % take an full acceleration.
    % The total traveled distances of the leader and the follower are also returned.
    distance_remaining = distance;
    time_to_catch = 0;
    waiting_time = 0; % time that the leader stops and waits for the follower
    distance_traveled_leader_total = 0;
    distance_traveled_follower_total = 0;
    speed_leader = mpa.trims(trim_leader).speed;
    speed_follower = mpa.trims(trim_follower).speed;

    % get the shortest path from the current trim to the equilibrium trim
    shortest_path_to_equilibrium = mpa.shortest_paths_to_equilibrium{trim_leader};

    % get the shortest path to the trims with the maximum speed
    shortest_path_to_max_speed = mpa.shortest_paths_to_max_speed{trim_follower};
    trim_max_speed = shortest_path_to_max_speed(end);
    max_speed = mpa.trims(trim_max_speed).speed;

    count_trim = 2; % start from the second trim in the path since the first one is the current trim

    distance_traveled = @(v0, a, t) v0 * t +1/2 * a * t^2; % calculate distance based on the initial speed and constant acceleration
    is_catch = false;

    while ~is_catch

        if speed_leader ~= 0
            % leader continue to decelerate
            trim_leader_next = shortest_path_to_equilibrium(count_trim);
            speed_leader_next = mpa.trims(trim_leader_next).speed;
            a_leader = (speed_leader_next - speed_leader) / time_step; % acceleration (negative value)
            % assert(a_leader<=0)
            % traveled distance of the current time step (assume linear acceleration)
            distance_traveled_leader_tmp = distance_traveled(speed_leader, a_leader, time_step);
        else
            speed_leader_next = 0;
            a_leader = 0;
            distance_traveled_leader_tmp = 0;
        end

        if speed_follower ~= max_speed
            % follower continue to accelerate
            trim_follower_next = shortest_path_to_max_speed(count_trim);
            speed_follower_next = mpa.trims(trim_follower_next).speed;
            a_follower = (speed_follower_next - speed_follower) / time_step; % acceleration (positive value)
            % assert(a_follower>=0)
            distance_traveled_follower_tmp = distance_traveled(speed_follower, a_follower, time_step);
        else
            speed_follower_next = max_speed;
            a_follower = 0;
            distance_traveled_follower_tmp = distance_traveled(speed_follower, a_follower, time_step);
        end

        % remaining distance for the follower to catch the leader
        distance_left_tmp = distance_remaining - distance_traveled_follower_tmp + distance_traveled_leader_tmp;

        if distance_left_tmp <= 0
            % the follower catchs the leader at this time step
            is_catch = true;
            % solve the quadratic equation to get the accumulating time of this time step
            r = roots([1/2 * (a_follower - a_leader), speed_follower - speed_leader, -distance_remaining]);
            assert(isreal(r) == true)
            time_accumulate = max(r); % choose the positive one
            % update the traveled distance of the current time step
            distance_traveled_leader_tmp = speed_leader + 0.5 * a_leader * time_accumulate^2;
            distance_traveled_follower_tmp = distance_traveled_leader_tmp + distance_remaining;

        else
            % accumulating time equals to the time step
            time_accumulate = time_step;
            % update the remaining distance
            distance_remaining = distance_left_tmp;
        end

        % update the STAC
        time_to_catch = time_to_catch + time_accumulate;
        % update the total traveled distance
        distance_traveled_leader_total = distance_traveled_leader_total + distance_traveled_leader_tmp;
        distance_traveled_follower_total = distance_traveled_follower_total + distance_traveled_follower_tmp;
        % update speeds
        speed_leader = speed_leader_next; % update the current speed for the next iteration
        speed_follower = speed_follower_next; % update the current speed for the next iteration
        % update the waiting time
        if speed_leader == 0 && speed_leader_next == 0
            waiting_time = waiting_time + time_accumulate;
        end

        % update counter
        count_trim = count_trim + 1;
    end

end
