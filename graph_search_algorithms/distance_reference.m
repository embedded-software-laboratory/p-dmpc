function result = distance_reference(init_pose, target_pose, cur_node)
    % syms t r
    % eqn = [init_pose.x, init_pose.y] + t * [target_pose.x, target_pose.y] == [pose.x, pose.y] + r * [target_pose.y , -target_pose.x];
    % [A, B] = equationsToMatrix([eqn], [t, r])
    % A = [target_pose.y, -target_pose.x;
    %     -target_pose.x, -target_pose.y];
    % B = [init_pose.x - cur_pose.x;
    %      init_pose.y - cur_pose.y];
    % X = linsolve(A, B);
    t = (cur_node.xs .* target_pose.xs + cur_node.ys .* target_pose.ys - init_pose.xs .* target_pose.xs - init_pose.ys .* target_pose.ys) ...
        ./(target_pose.xs .^2 + target_pose.ys .^2);
    solutions = [init_pose.xs; init_pose.ys] + t .* [target_pose.xs; target_pose.ys];
    reference_poses.xs = solutions(1,:);
    reference_poses.ys = solutions(2,:);
    result = double(euclidean_distance(cur_node, reference_poses));
end

