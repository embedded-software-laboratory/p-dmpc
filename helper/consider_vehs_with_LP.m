function scenario_v = consider_vehs_with_LP(scenario_v, iter, vehicle_idx, all_coupling_vehs_without_ROW)
% CONSIDER_VEHS_WITH_LP Stategies to let vehicle with the right-of-way
% consider vehicle without the right-of-way 
% '1': do not consider 
% '2': consider currently occupied area as static obstacle
% '3': consider the occupied area of emergency braking maneuver as static obstacle 
% '4': consider one-step reachable sets as static obstacle
% '5': consider old trajectory as dynamic obstacle
    
    for i_LP = 1:length(all_coupling_vehs_without_ROW)
        veh_without_ROW = all_coupling_vehs_without_ROW(i_LP);
        
        % stategies to let vehicle with the right-of-way consider vehicle without the right-of-way
        switch scenario_v.options.strategy_consider_veh_without_ROW
            case '1'
                % do not consider

            case '2'
                % consider currently occupied area as static obstacle
                scenario_v.obstacles{end+1} = iter.occupied_areas{veh_without_ROW}.normal_offset; % add as static obstacles

            case '3'
                % consider the occupied area of emergency braking maneuver
                % as static obstacle (only if their couplings are not
                % ignored by forbidding one vehicle entering their lanelet
                % crossing area, and they have side-impact collision
                % possibility). Cases that vehicles drive successively are not
                % included to avoid that vehicles behind push vehicles in
                % front to move forward.
                switch scenario_v.options.priority
                    case 'right_of_way_priority'
                        find_coupling = [scenario_v.coupling_info.veh_with_ROW]==vehicle_idx & [scenario_v.coupling_info.veh_without_ROW]==veh_without_ROW;
                        if scenario_v.k==261
                            disp('')
                        end
                        if ~scenario_v.coupling_info(find_coupling).is_ignored && strcmp(scenario_v.coupling_info(find_coupling).collision_type,CollisionType.type_2) ...
                                && strcmp(scenario_v.coupling_info(find_coupling).lanelet_relationship, LaneletRelationshipType.type_5)
                            % the emergency braking maneuver is only considered if
                            % two coupled vehicles at crossing-adjacent lanelets have side-impact collision that is not ignored
                            scenario_v.obstacles{end+1} = iter.emergency_maneuvers{veh_without_ROW}.braking_area;
                        else
                            scenario_v.obstacles{end+1} = iter.occupied_areas{veh_without_ROW}.normal_offset;
                        end
                    otherwise
                        scenario_v.obstacles{end+1} = iter.emergency_maneuvers{veh_without_ROW}.braking_area;
                end

            case '4'
                % consider one-step reachable sets as static obstacle
                reachable_sets = iter.reachable_sets{veh_without_ROW,1};
                % get boundary of the polygon
                [x_reachable_sets, y_reachable_sets] = boundary(reachable_sets);
                scenario_v.obstacles(end+1) = {[x_reachable_sets';y_reachable_sets']};
            case '5'
                % consider old trajectory as dynamic obstacle
                latest_msg = scenario_v.ros_subscribers{veh_without_ROW}.LatestMessage;
                if latest_msg.time_step > 0
                    % the message does not come from the initial time step
                    predicted_areas = arrayfun(@(array) {[array.x';array.y']}, latest_msg.predicted_areas);
                    shift_step = scenario_v.k - latest_msg.time_step; % times that the prediction should be shifted and the last prediction should be repeated
                    if shift_step > 1
                        disp(['shift step is ' num2str(shift_step) ', ego vehicle: ' num2str(vehicle_i) ', considered vehicle: ' num2str(veh_without_ROW)])
                    end
                    predicted_areas = del_first_rpt_last(predicted_areas(:)', shift_step);
                    scenario_v.dynamic_obstacle_area(end+1,:) = predicted_areas;
                end
            otherwise
                warning("Please specify one of the following strategies to let vehicle with a higher priority also consider vehicle with a lower priority: '1', '2', '3', '4', '5'.")
        end

    end
end