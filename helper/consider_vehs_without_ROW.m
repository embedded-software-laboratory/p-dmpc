function scenario_v = consider_vehs_without_ROW(scenario_v, iter, all_coupling_vehs_without_ROW)
% CONSIDER_VEHS_WITHOUT_ROW Stategies to let vehicle with the right-of-way
% consider vehicle without the right-of-way 
% '0': do not consider 
% '1': consider currently occupied area as static obstacle
% '2': consider one-step reachable sets as static obstacle
% '3': consider old trajectory as dynamic obstacle
% '4': consider the occupied area of emergency braking maneuver as static obstacle 
    
    for i_LP = 1:length(all_coupling_vehs_without_ROW)
        veh_without_ROW = all_coupling_vehs_without_ROW(i_LP);
        
        % stategies to let vehicle with the right-of-way consider vehicle without the right-of-way
        switch scenario_v.strategy_consider_veh_without_ROW
            case '0'
                % do not consider

            case '1'
                % consider currently occupied area as static obstacle
                scenario_v.obstacles{end+1} = iter.occupied_areas{veh_without_ROW}.normal_offset; % add as static obstacles

            case '2'
                % consider one-step reachable sets as static obstacle
                reachable_sets = iter.reachable_sets{veh_without_ROW,1};
                % get boundary of the polygon
                [x_reachable_sets, y_reachable_sets] = boundary(reachable_sets);
                scenario_v.obstacles(end+1) = {[x_reachable_sets';y_reachable_sets']};

            case '3'
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
                
            case '4'
                % consider the occupied area of emergency braking maneuver (with normal offset) as static obstacle 
                scenario_v.obstacles{end+1} = iter.emergency_braking_maneuvers{veh_without_ROW}.area;
            otherwise
                warning("Please specify one of the following strategies to let vehicle with a higher priority also consider vehicle with a lower priority: '0', '1', '2', '3', '4'.")
        end

    end
end