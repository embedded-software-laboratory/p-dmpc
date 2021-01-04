function costs = cost_to_go(situation_costs, init_poses, target_poses, next_node)

    ref_poses = reference_pose(init_poses, target_poses, next_node);
    remaining_costs = euclidean_distance(ref_poses, next_node);
    yaws = wrapToHalfPi(next_node.yaws) - wrapToHalfPi(ref_poses.yaws);
    for i = 1:length(yaws)
        [value, index] = min(abs(situation_costs.angle - yaws(i)));
        yaws(i) = situation_costs.angle(index);
        
        if remaining_costs(i) > abs(situation_costs.dy(index))
            (remaining_costs(i) - situation_costs.avg_distance(index)) * situation_costs.iterations(index);
            remaining_costs(i) - situation_costs.dy(index);
        end
    end
    
    costs = remaining_costs;
    while any(remaining_costs > 1.6)
        remaining_costs = max(remaining_costs - 1.6, 0);
        costs = costs + remaining_costs;
    end
end

