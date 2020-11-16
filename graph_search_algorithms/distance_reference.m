function result = distance_reference(init_pose, target_pose, cur_pose)
    % syms t r
    % eqn = [init_pose.x, init_pose.y] + t * [target_pose.x, target_pose.y] == [pose.x, pose.y] + r * [target_pose.y , -target_pose.x];
    % [A, B] = equationsToMatrix([eqn], [t, r])
    % A = [target_pose.y, -target_pose.x;
    %     -target_pose.x, -target_pose.y];
    % B = [init_pose.x - cur_pose.x;
    %      init_pose.y - cur_pose.y];
    % X = linsolve(A, B);
    t = (cur_pose.x*target_pose.x + cur_pose.y*target_pose.y - init_pose.x*target_pose.x - init_pose.y*target_pose.y)/(target_pose.x^2 + target_pose.y^2);
    solution = [init_pose.x, init_pose.y] + t * [target_pose.x, target_pose.y];
    reference_pose.x = solution(1);
    reference_pose.y = solution(2);
    result = double(euclidean_distance(reference_pose, cur_pose));
end

