function [g_values, h_values] = calculate_next_values_reference(g_values, situation_costs, init_poses, target_poses, next_node)
    distance = distance_reference(init_poses, target_poses, next_node);
    g_values = g_values + distance;
    h_values = cost_to_go(situation_costs, init_poses, target_poses, next_node);
end