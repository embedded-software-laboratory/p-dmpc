function [gvalue, hvalue] = calculate_next_values_distance(parent_gvalue, last_pose, next_pose, target_pose)

    gvalue_change = euclidean_distance(last_pose,next_pose);
            
    gvalue = parent_gvalue + gvalue_change;

    hvalue = euclidean_distance(next_pose, target_pose);
    
end

