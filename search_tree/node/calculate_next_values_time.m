function [g_value, h_value] = calculate_next_values_time(parent_g_value, next_pose, target_pose)
    g_value = parent_g_value + dt;
    h_value = euclidean_distance(next_pose, target_pose)/v_max;
end

