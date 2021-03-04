function [leaf_nodes, candidates, search_tree, max_id] = expand_horizon(leaf_nodes, search_tree, next_id, obstacles, motion_graph, situation_costs, trim_length, init_poses, target_poses, visited_nodes, max_id)
    
    n_veh = length(motion_graph.motionGraphList);
    cur_node = search_tree.Node{next_id};
    trim_tuple = motion_graph.trimTuple;
    candidates = [];
    is_goals = is_goal(cur_node, target_poses);
    
    cur_trim_id = tuple2index(cur_node.trims,trim_length);
    successor_trim_ids = find(motion_graph.transitionMatrix(cur_trim_id, 1:length(trim_tuple) - 1));
    next_node = cur_node;
    if cur_node.depth < h_u
        for id = successor_trim_ids
            next_node.trims = trim_tuple(id,:);
            [leaf_nodes, candidates, search_tree, max_id] = update_horizon(cur_node, next_node, leaf_nodes, candidates, search_tree, next_id, ...
                                                                        obstacles, motion_graph, situation_costs, init_poses, target_poses, visited_nodes, max_id, is_goals);
        end
    elseif cur_node.depth < h_p
        [leaf_nodes, candidates, search_tree, max_id] = update_horizon(cur_node, next_node, leaf_nodes, candidates, search_tree, next_id, ...
                                                                    obstacles, motion_graph, situation_costs, init_poses, target_poses, visited_nodes, max_id, is_goals);
    end
end