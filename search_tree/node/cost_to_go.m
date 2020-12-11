function cost = cost_to_go(init_pose, target_pose, next_pose)
    ref_pose = reference_pose(init_pose, target_pose, next_pose);
    remaining_cost = euclidean_distance(ref_pose, next_pose);
    cost = remaining_cost;
    while any(remaining_cost > 1.6)
        remaining_cost = max(remaining_cost - 1.6, 0);
        cost = cost + remaining_cost;
    end
end

