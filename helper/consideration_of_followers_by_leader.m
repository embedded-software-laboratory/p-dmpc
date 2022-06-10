function scenario_v = consideration_of_followers_by_leader(scenario_v, iter, all_coupling_vehs_LP)
% CONSIDERATION_OF_FOLLOWERS_BY_LEADER Currently five stategies are
% supported to let vehicle with a higher priority consider vehicle with a
% lower priority
% '0': do not consider 
% '1': consider currently occupied area as static obstacle
% '2': consider one-step reachable sets as static obstacle
% '3': consider old trajectory as dynamic obstacle
% '4': consider the occupied area of emergency braking maneuver as static obstacle 
    for i_LP = 1:length(all_coupling_vehs_LP)
        veh_LP = all_coupling_vehs_LP(i_LP);
        
        % let vehicle with a higher priority also consider vehicle with a lower priority
        switch scenario_v.strategy_consider_veh_with_lower_prio
            case '0'
                % do not consider

            case '1'
                % current occupied area as static obstacle
                veh = Vehicle();
                % calculate the current occupied area 
                [x_globals,y_globals] = translate_global(...
                    iter.x0(veh_LP, indices().heading),...              % yaw angle
                    iter.x0(veh_LP, indices().x),...                    % x-position
                    iter.x0(veh_LP, indices().y),...                    % y-position
                    [-1,-1, 1, 1]*(veh.Length/2+scenario_v.offset),...  % x-coordinates of the local occupied area 
                    [-1, 1, 1,-1]*(veh.Width/2+scenario_v.offset));     % y-coordinates of the local occupied area 
                
                scenario_v.obstacles(end+1) = {[x_globals;y_globals]}; % add as static obstacles

            case '2'
                % one-step reachable set as static obstacle
                reachable_sets_LP = iter.reachable_sets{veh_LP,1};
                % turn polyshape to plain array
                reachable_sets_LP_array = {[reachable_sets_LP.Vertices(:,1)';reachable_sets_LP.Vertices(:,2)']};
                scenario_v.obstacles(end+1) = reachable_sets_LP_array;
            case '3'
                % old trajectory as dynamic obstacle
                latest_msg_LP = scenario_v.ros_subscribers{veh_LP}.LatestMessage;
                if latest_msg_LP.time_step > 0
                    % the message does not come from the initial time step
                    predicted_areas_LP = arrayfun(@(array) {[array.x';array.y']}, latest_msg_LP.predicted_areas);
                    shift_step = scenario_v.k - latest_msg_LP.time_step; % times that the prediction should be shifted and the last prediction should be repeated
                    if shift_step > 1
                        disp(['shift step is ' num2str(shift_step) ', ego vehicle: ' num2str(vehicle_i) ', considered vehicle: ' num2str(veh_LP)])
                    end
                    predicted_areas_LP = del_first_rpt_last(predicted_areas_LP(:)', shift_step);
                    scenario_v.dynamic_obstacle_area(end+1,:) = predicted_areas_LP;
                end
            case '4'
                % consider the occupied area of emergency braking maneuver as static obstacle
                yaw_LP = iter.x0(veh_LP,indices().heading);
                x_LP = iter.x0(veh_LP,indices().x);
                y_LP = iter.x0(veh_LP,indices().y);
                trim_current_LP = iter.trim_indices(veh_LP);
                shortest_path_to_equilibrium = get_shortest_path_to_equilibrium(scenario_v.mpa, trim_current_LP);
                if length(shortest_path_to_equilibrium)==1
                    % vehicle already at the equilibrium trim
                    trim_next_LP = trim_current_LP;
                else
                    trim_next_LP = shortest_path_to_equilibrium(2);
                end
                
                % local shape to global shape
                maneuver = scenario_v.mpa.maneuvers{trim_current_LP,trim_next_LP};
                [shape_x,shape_y] = translate_global(yaw_LP,x_LP,y_LP,maneuver.area(1,:),maneuver.area(2,:));
                shape_braking = {[shape_x;shape_y]};
                scenario_v.obstacles(end+1) = shape_braking;
            otherwise
                warning("Please specify one of the following strategies to let vehicle with a higher priority also consider vehicle with a lower priority: '0', '1', '2', '3', '4'.")
        end

    end
end