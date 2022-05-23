function scenario_v = vehicles_reachable_sets_as_obstacles(scenario_v, iter_filtered_other_grps, coupled_with_other_grps_HP)
% VEHICLES_REACHABLE_SETS_AS_OBSTACLES Add reachable sets of the coupled vehicles with
%                       higher priorities in other groups as obsticle
% Params:
%     dynamic_obstacle_reachableSets_old:
%         Reachable sets of previous obstacles
%     iter_filtered_other_grps:
%         Iteration information about coupling vehicles in other groups
%     local_reachable_sets_other_grps:
%         The reachable sets in the prediction horizon calculated offline using motion primitives
% Returns:
%     dynamic_obstacle_reachableSets: cell [numberOfCoupledVehiclesInOtherGroups x Hp]
%         

    % get reachable sets of coupled vehicles with higher priorities in the other groups
    for idx_i = 1:length(coupled_with_other_grps_HP)
        veh_i = coupled_with_other_grps_HP(idx_i);
        latest_msg_i = scenario_v.ros_subscribers{veh_i}.LatestMessage;
        if latest_msg_i.time_step == scenario_v.k
            % if the latest message comes from current time step, the
            % coupling will be considered by set the predicted areas as dynamic obstacles 
            predicted_areas = arrayfun(@(array) {[array.x';array.y']}, latest_msg_i.predicted_areas);
            scenario_v.dynamic_obstacle_area(end+1,:) = predicted_areas;
        else
            % otherwise, the reachable sets calculated at the begining of
            % the current time step, which is based on the messages sent in
            % previous time step, will be directly used to save computation time
            reachable_sets_i = iter_filtered_other_grps.reachable_sets(idx_i,:);
            % delete the first reachable set since it is the reachable set of the current but not the next time step, and repeat the last reachable set
            reachable_sets_i = del_first_rpt_last(reachable_sets_i);

            % convexify the reachable sets since the separate axis theorem works only for convex shapes
            points_idx = cellfun(@(cell) {convhull(cell.Vertices,'Simplify',true)}, reachable_sets_i);
            reachable_sets_conv_i = cellfun(@(reachable_set, idx) {[reachable_set.Vertices(idx,1)';reachable_set.Vertices(idx,2)']}, reachable_sets_i, points_idx);
            % extract data from polyshape
    %         cellfun(@(cell) {[cell.Vertices(:,1)';cell.Vertices(:,2)']}, reachable_sets_i)
    
            % add reachable sets as dynamic obstacles
            scenario_v.dynamic_obstacle_reachableSets(end+1,:) = reachable_sets_conv_i;
        end
    end

end
