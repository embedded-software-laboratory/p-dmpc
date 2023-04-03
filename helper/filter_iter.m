function iter = filter_iter(iter, vehicle_filter) % TODO: Add new fields
    % FILTER_ITER   Reduce iter structure to selected vehicles.

    iter.x0 = iter.x0(vehicle_filter, :);
    iter.trim_indices = iter.trim_indices(vehicle_filter);
    iter.v_ref = iter.v_ref(vehicle_filter, :);
    iter.referenceTrajectoryPoints = iter.referenceTrajectoryPoints(vehicle_filter, :, :);
    iter.referenceTrajectoryIndex = iter.referenceTrajectoryIndex(vehicle_filter, :, :);
    iter.predicted_lanelets = iter.predicted_lanelets(vehicle_filter);
    iter.predicted_lanelet_boundary = iter.predicted_lanelet_boundary(vehicle_filter, :);
    iter.reachable_sets = iter.reachable_sets(vehicle_filter, :);
    iter.emergency_maneuvers = iter.emergency_maneuvers(vehicle_filter);
    iter.last_trajectory_index = iter.last_trajectory_index(vehicle_filter);
    iter.lane_change_lanes = iter.lane_change_indices(vehicle_filter, :, :);
    iter.lane_change_indices = iter.lane_change_indices(vehicle_filter, :, :);
    iter.lanes_before_update = iter.lanes_before_update(vehicle_filter, :, :);
    iter.auto_updated_path = iter.auto_updated_path(vehicle_filter);
    iter.vehicle_to_lanelet = iter.vehicle_to_lanelet(vehicle_filter);
    iter.vehicles = iter.vehicles(vehicle_filter);
    iter.amount = sum(vehicle_filter);
    iter.hdv_adjacency = iter.hdv_adjacency(vehicle_filter, :);
end
