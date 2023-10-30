function iter = filter_iter(iter, vehicle_filter) % TODO: Add new fields
    % FILTER_ITER   Reduce iter structure to selected vehicles.

    iter.x0 = iter.x0(vehicle_filter, :);
    iter.trim_indices = iter.trim_indices(vehicle_filter);
    iter.v_ref = iter.v_ref(vehicle_filter, :);
    iter.reference_trajectory_points = iter.reference_trajectory_points(vehicle_filter, :, :);
    iter.reference_trajectory_index = iter.reference_trajectory_index(vehicle_filter, :, :);
    iter.predicted_lanelets = iter.predicted_lanelets(vehicle_filter);
    iter.predicted_lanelet_boundary = iter.predicted_lanelet_boundary(vehicle_filter, :);
    iter.reachable_sets = iter.reachable_sets(vehicle_filter, :);
    iter.lanelet_crossing_areas = iter.lanelet_crossing_areas{vehicle_filter};
    iter.vehicle_to_lanelet = iter.vehicle_to_lanelet(vehicle_filter);
    iter.vehicles = iter.vehicles(vehicle_filter);
    iter.vehicle_ids = iter.vehicle_ids(vehicle_filter);
    iter.amount = sum(vehicle_filter);
    iter.hdv_adjacency = iter.hdv_adjacency(vehicle_filter, :);
end
