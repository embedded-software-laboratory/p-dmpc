function [leaf_nodes, candidates, search_tree, max_id] = expand_node(scenario, iter, leaf_nodes, search_tree, next_id, trim_length, visited_nodes, max_id)
    cur_node = search_tree.Node{next_id};
    trim_tuple = scenario.combined_graph.trimTuple;
    candidates = [];
    is_goals = is_goal(cur_node, scenario);
    
    cur_trim_id = tuple2index(cur_node.trims,trim_length);
    if cur_node.depth < h_u
        successor_trim_ids = find(scenario.combined_graph.transitionMatrix(cur_trim_id, :));
    else % h_u <= cur_node.depth < h_p
        successor_trim_ids = cur_trim_id;
    end
    next_node = cur_node;

    for id = successor_trim_ids
        next_node.trims = trim_tuple(id,:);
        [leaf_nodes, candidates, search_tree, max_id] = eval_child(scenario, iter, cur_node, next_node, leaf_nodes, candidates, search_tree, next_id, ...
                                                                    visited_nodes, max_id, is_goals);
    end
end