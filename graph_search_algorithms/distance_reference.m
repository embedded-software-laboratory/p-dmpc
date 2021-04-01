function result = distance_reference(init_poses, target_poses, cur_node)
    % syms t r
    % eqn = [init_pose.x, init_pose.y] + t * [target_pose.x, target_pose.y] == [pose.x, pose.y] + r * [target_pose.y , -target_pose.x];
    % [A, B] = equationsToMatrix([eqn], [t, r])
    % A = [target_pose.y, -target_pose.x;
    %     -target_pose.x, -target_pose.y];
    % B = [init_pose.x - cur_pose.x;
    %      init_pose.y - cur_pose.y];
    % X = linsolve(A, B);
    % TODO delete function
    t = (cur_node.xs .* target_poses.xs + cur_node.ys .* target_poses.ys - init_poses.xs .* target_poses.xs - init_poses.ys .* target_poses.ys) ...
        ./(target_poses.xs .^2 + target_poses.ys .^2);
    solutions = [init_poses.xs; init_poses.ys] + t .* [target_poses.xs; target_poses.ys];
    reference_poses.xs = solutions(1,:);
    reference_poses.ys = solutions(2,:);
    result = double(euclidean_distance(cur_node, reference_poses));
end

