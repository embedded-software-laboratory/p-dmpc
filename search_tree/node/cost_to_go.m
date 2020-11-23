function cost = cost_to_go(init_pose, target_pose, next_pose)
    ref_pose = reference_pose(init_pose, target_pose, next_pose);
    remaining_cost = euclidean_distance(ref_pose, next_pose);
    cost = remaining_cost;
    while remaining_cost > 1.6
        remaining_cost = remaining_cost - 1.6;
        cost = cost + remaining_cost;
    end
    cost = cost + remaining_cost;
end

