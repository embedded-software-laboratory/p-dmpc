function [weighted_coupling_reduced, coupling_info, lanelet_crossing_areas] = reduce_coupling_lanelet_crossing_area(iter, strategy_enter_lanelet_crossing_area)
    % This function implement the strategies of letting vehicle enter the crossing area, which is the overlapping area of two
    % vehicles' lanelet boundaries. Four strategies are existed.

    % Reduced coupling weights after forbidding vehicles entering their lanelet crossing areas
    weighted_coupling = iter.weighted_coupling;
    weighted_coupling_reduced = weighted_coupling;
    coupling_info = iter.coupling_info;
    nVeh = size(coupling_info, 1);
    lanelet_crossing_areas = cell(nVeh, 1);

    % coupling info is empty if it is no commonroad scenario
    if isempty([coupling_info{:}])
        return
    end

    [rows, cols] = find(weighted_coupling);

    for veh_pair = 1:length(rows)

        veh_i = rows(veh_pair);
        veh_j = cols(veh_pair);

        collision_type = iter.coupling_info{veh_i, veh_j}.collision_type;
        lanelet_relationship = iter.coupling_info{veh_i, veh_j}.lanelet_relationship;

        % only side-impact collision should be ignore by this strategy
        if collision_type == CollisionType.from_rear
            continue
        end

        is_at_intersection = all((coupling_info{veh_i, veh_j}.is_intersection));
        is_intersecting_lanelets = (lanelet_relationship == LaneletRelationshipType.crossing);
        is_merging_lanelets = (lanelet_relationship == LaneletRelationshipType.merging);

        % check if coupling edge should be ignored
        switch strategy_enter_lanelet_crossing_area
            case '1'
                % no constraint on entering the crossing area
                return
            case '2'
                % not allowed to enter the crossing area if they are coupled at intersecting lanelets of the intersection
                is_ignore_coupling = is_intersecting_lanelets && is_at_intersection;
            case '3'
                % not allowed to enter the crossing area if they are coupled at intersecting or merging lanelets of the intersection
                is_ignore_coupling = (is_intersecting_lanelets || is_merging_lanelets) && is_at_intersection;
            case '4'
                % not allowed to enter the crossing area if they are coupled at intersecting or merging lanelets regardless whether they are at the intersection or not
                is_ignore_coupling = is_intersecting_lanelets || is_merging_lanelets;
            otherwise
                warning("Please specify one of the following strategies to let vehicle enter crossing area: '0', '1', '2', '3'.")
                return
        end

        if is_ignore_coupling
            % check if vehicle without right-of-way has already enter the crossing area

            % get the crossing area of two vehicles' lanelet
            lanelet_crossing_area = intersect(iter.predicted_lanelet_boundary{veh_i, 3}, iter.predicted_lanelet_boundary{veh_j, 3});

            [lanelet_crossing_area_x, lanelet_crossing_area_y] = boundary(lanelet_crossing_area);
            lanelet_crossing_area_xy = [lanelet_crossing_area_x, lanelet_crossing_area_y]';

            % check if at least one emergency (braking/left turn/right turn) maneuver is collision-free with lanelet crossing area
            if any(inpolygon(iter.occupied_areas{veh_j}.without_offset(1, :), iter.occupied_areas{veh_j}.without_offset(2, :), lanelet_crossing_area_x, lanelet_crossing_area_y)) || ... % ensure vehicle is not in the lanelet crossing area
                    (InterX(iter.emergency_maneuvers{veh_j}.braking_area_without_offset, lanelet_crossing_area_xy) && ... % check if emergency braking maneuver could avoid the LCA
                    InterX(iter.emergency_maneuvers{veh_j}.left_area_without_offset, lanelet_crossing_area_xy) && ... % check if emergency left-turn maneuver could avoid the LCA
                    InterX(iter.emergency_maneuvers{veh_j}.right_area_without_offset, lanelet_crossing_area_xy)) % check if emergency right-turn maneuver could avoid the LCA
                % The vehicle without the ROW must be allowed to enter the lanelet crossing area since all its emergency meneuvers
                % will collide with this area. In this case, we check if vehicle with the ROW could be forbidded to enter.
                if any(inpolygon(iter.occupied_areas{veh_i}.without_offset(1, :), iter.occupied_areas{veh_i}.without_offset(2, :), lanelet_crossing_area_x, lanelet_crossing_area_y)) || ... % ensure vehicle is not in the lanelet crossing area
                        (InterX(iter.emergency_maneuvers{veh_i}.braking_area_without_offset, lanelet_crossing_area_xy) && ... % check if emergency braking maneuver could avoid the LCA
                        InterX(iter.emergency_maneuvers{veh_i}.left_area_without_offset, lanelet_crossing_area_xy) && ... % check if emergency left-turn maneuver could avoid the LCA
                        InterX(iter.emergency_maneuvers{veh_i}.right_area_without_offset, lanelet_crossing_area_xy)) % check if emergency right-turn maneuver could avoid the LCA
                    % The vehicle with the ROW must also be allowed to enter
                    % disp(['Both vehicle ' num2str(veh_i) ' and ' num2str(veh_j) ' have entered the crossing area, thus the coupling cannot be ignored.'])
                    continue
                else
                    % vehicle with the ROW could be forbidded to enter while vehicle without the ROW could not: swap their ROW and forbid vehicle with the ROW to enter
                    % disp(['Swap right-of-way: vehicle ' num2str(veh_j) ' now has right-of-way over ' num2str(veh_i) '.'])
                    veh_forbid = veh_i;
                    veh_free = veh_j;
                end

            else
                % forbid vehicle without the ROW to enter
                veh_forbid = veh_j;
                veh_free = veh_i;
            end

            % disp(['Ignore the coupling from vehicle ' num2str(veh_free) ' to ' num2str(veh_forbid) ' by forbidding the latter to enter the crossing area of their lanelets.'])
            coupling_info{veh_free, veh_forbid}.is_ignored = true; % ignore coupling since no collision is possible anymore
            coupling_info{veh_forbid, veh_free} = {}; % remove coupling info

            weighted_coupling_reduced(veh_i, veh_j) = 0;
            % store lanelet crossing area for later use
            for i_region = 1:lanelet_crossing_area.NumRegions
                [x_tmp, y_tmp] = boundary(lanelet_crossing_area, i_region);
                lanelet_crossing_areas{veh_forbid}(end + 1) = {[x_tmp'; y_tmp']};
            end

        end

    end

end