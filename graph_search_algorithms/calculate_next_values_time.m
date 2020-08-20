function [gvalue, hvalue] = calculate_next_values_time(parent_gvalue, max_velocity, next_pose, target_pose)

    gvalue_change = 1;
            
    gvalue = parent_gvalue + gvalue_change;

    hvalue = euclidean_distance(next_pose, target_pose)/max_velocity;
    
end

