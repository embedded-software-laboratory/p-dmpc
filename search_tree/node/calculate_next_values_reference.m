function [g_value, h_value] = calculate_next_values_reference(h_value, init_pose, target_pose, next_pose)
    g_value = 0;
    h_value = mean([h_value, distance_reference(init_pose, target_pose, next_pose)]);
end