function [g_value, h_value] = calculate_next_values_time(parent_g_value, max_velocity, next_pose, target_pose)

    g_value_change = 1;
            
    g_value = parent_g_value + g_value_change;

    h_value = euclidean_distance(next_pose, target_pose)/max_velocity;
    
end

