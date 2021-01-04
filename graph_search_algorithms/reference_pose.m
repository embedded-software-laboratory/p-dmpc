function reference_poses = reference_pose(init_pose, target_pose, cur_node)
    t = (cur_node.xs .* target_pose.xs + cur_node.ys .* target_pose.ys - init_pose.xs .* target_pose.xs - init_pose.ys .* target_pose.ys) ...
        ./(target_pose.xs .^2 + target_pose.ys .^2);
    solutions = [init_pose.xs; init_pose.ys] + t .* [target_pose.xs; target_pose.ys];
    reference_poses.xs = solutions(1,:);
    reference_poses.ys = solutions(2,:);
    reference_poses.yaws = init_pose.yaws;
end

