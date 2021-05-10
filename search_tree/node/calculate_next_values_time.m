function [g_value, h_value] = calculate_next_values_time(scenario, parent_g_value, next_pose)
    g_value = parent_g_value + dt;
    % TODO v_max from iter.vRef(iVeh)
    h_value = euclidean_distance(
        next_pose.xs, next_pose.ys...
        [scenario.vehicles(:).x_goal],[scenario.vehicles(:).y_goal])...
        /v_max;
end

