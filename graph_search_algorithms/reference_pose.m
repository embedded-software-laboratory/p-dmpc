function reference_pose = reference_pose(init_pose, target_pose, cur_pose)
    t = (cur_pose.x*target_pose.x + cur_pose.y*target_pose.y - init_pose.x*target_pose.x - init_pose.y*target_pose.y)/(target_pose.x^2 + target_pose.y^2);
    solution = [init_pose.x, init_pose.y] + t * [target_pose.x, target_pose.y];
    reference_pose.x = solution(1);
    reference_pose.y = solution(2);
end

