function [g_value, h_value] = calculate_next_values_reference(parent_g_value, init_pose, target_pose, next_pose)
    g_value = parent_g_value + dt;
    h_value = distance_reference(init_pose, target_pose, next_pose);
end