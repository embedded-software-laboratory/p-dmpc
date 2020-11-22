function [leaf_nodes, search_tree, max_id, is_goals] = expand_horizon(leaf_nodes, search_tree, next_id, motion_graph, trim_length, init_poses, target_poses, visited_nodes, max_id, is_goals)
    
    n_veh = length(motion_graph.motionGraphList);
    cur_node = search_tree.Node{next_id};
    trim_tuple = motion_graph.trimTuple;
    trim_tuple_size = length(trim_tuple);
    cur_trim_id = tuple2index(cur_node.trims,trim_length);
    
    if cur_node.depth < h_u
        for i = 1 : trim_tuple_size

            next_node = cur_node;
            next_node.trims = trim_tuple(i,:);
            next_trim_id = tuple2index(next_node.trims,trim_length);

            existsManeuver = (motion_graph.transitionMatrix(cur_trim_id,next_trim_id) == 1);
            if ~existsManeuver
                continue;
            end

            [leaf_nodes, search_tree, max_id, is_goals] = update_horizon(cur_node, next_node, leaf_nodes, search_tree, next_id, motion_graph, init_poses, target_poses, visited_nodes, max_id, is_goals);

            if sum(is_goals) == n_veh
                return;
            end
        end
    elseif cur_node.depth < h_p
        next_node = cur_node;
        [leaf_nodes, search_tree, max_id, is_goals] = update_horizon(cur_node, next_node, leaf_nodes, search_tree, next_id, motion_graph, init_poses, target_poses, visited_nodes, max_id, is_goals);
    end
end