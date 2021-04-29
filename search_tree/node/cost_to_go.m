function costs = cost_to_go(situation_costs, init_poses, target_poses, next_node, motion_graph)
    % TODO delete
    ref_poses = reference_pose(init_poses, target_poses, next_node);
    yaws = wrapToHalfPi(next_node.yaws) - wrapToHalfPi(ref_poses.yaws);
    n_veh = length(yaws);
    level = next_node.depth * ones(1, n_veh);
    remaining_costs = euclidean_distance(ref_poses, next_node);
    costs = remaining_costs;
    if(next_node.depth < h_u)
        for i = 1:n_veh
            diff_values = situation_costs.angle - yaws(i);
            diff_values(diff_values > 0) = -inf;
            [~, index] = max(diff_values);
            yaws(i) = situation_costs.angle(index);

            if remaining_costs(i) > abs(situation_costs.dy(index))
                costs(i) = (remaining_costs(i) - situation_costs.avg_distance(index)) * min(h_p - level(i), situation_costs.iterations(index));
                remaining_costs(i) = remaining_costs(i) - situation_costs.dy(index);
                level(i) = level(i) + situation_costs.iterations(index);
            end
        end

        while any(remaining_costs > 1.6 & level < h_p)
            remaining_costs = max(remaining_costs - 1.6, 0);
            costs = costs + (level < h_p) .* remaining_costs;
            level = level + 1;
        end
    else
        for depth = next_node.depth : (h_p - 1)
            for i = 1:n_veh
                maneuver = motion_graph.motionGraphList(i).maneuvers{next_node(iVeh,idx.trim), next_node(iVeh,idx.trim)};
                [next_node(iVeh,idx.x), next_node(iVeh,idx.y)] = translate_global(next_node(iVeh,idx.yaw), next_node(iVeh,idx.x), next_node(iVeh,idx.y), maneuver.dx, maneuver.dy);
            end
            ref_poses = reference_pose(init_poses, target_poses, next_node);
            remaining_costs = euclidean_distance(ref_poses, next_node);
            costs = costs + remaining_costs;
        end
    end
end

function reference_poses = reference_pose(init_pose, target_pose, cur_node)
    t = (cur_node.xs .* target_pose.xs + cur_node.ys .* target_pose.ys - init_pose.xs .* target_pose.xs - init_pose.ys .* target_pose.ys) ...
        ./(target_pose.xs .^2 + target_pose.ys .^2);
    solutions = [init_pose.xs; init_pose.ys] + t .* [target_pose.xs; target_pose.ys];
    reference_poses.xs = solutions(1,:);
    reference_poses.ys = solutions(2,:);
    reference_poses.yaws = init_pose.yaws;
end

