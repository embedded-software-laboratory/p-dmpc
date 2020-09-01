function [g_value, h_value] = calculate_next_values_distance(parent_g_value, last_pose, next_pose, target_pose)

    g_value_change = euclidean_distance(last_pose,next_pose);
            
    g_value = parent_g_value + g_value_change;

    h_value = euclidean_distance(next_pose, target_pose);
    
end

