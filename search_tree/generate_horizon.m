function [search_tree, leaf_nodes] = generate_horizon(init_poses, target_poses, cur_node, trim_indices, combined_graph)

    n_veh = length(combined_graph.motionGraphList);
    trim_length = zeros(1, n_veh);
    for i = 1 : n_veh
        trim_length(i) = length(combined_graph.motionGraphList(i).trims);
    end
    
    % Create tree with root node
    search_tree = tree(node(0, trim_indices, cur_node.xs, cur_node.ys, cur_node.yaws, cur_node.g_values, cur_node.h_values));
    
    % Array storing ids of nodes that may be expanded
    id = 1;
    leaf_nodes = [id];
    next_node_id = id;
    
    % Array storing ids of nodes that were visited
    visited_nodes = [];
    
    % Initialize
    cur_poses = node2poses(cur_node);
    is_goals = is_goal(cur_poses, target_poses);
    
    % Expand leaves of tree until depth or target is reached or until there 
    % are no leaves
    while (((search_tree.Node{id}.depth < h_p) && ~(sum(is_goals) == n_veh) && ~isempty(leaf_nodes)) || cheaper(search_tree.Node{next_node_id}, search_tree.Node{id}))
               
        % Delete chosen entry from list of expandable nodes
        leaf_nodes(leaf_nodes == next_node_id) = [];
        [leaf_nodes, search_tree, id, is_goals] = expand_horizon(leaf_nodes, search_tree, next_node_id, combined_graph, ...
                                                              trim_length, init_poses, target_poses, visited_nodes, id, is_goals);
        
        visited_nodes = [visited_nodes, next_node_id];       
        
        % get next node for expansion
        next_node_id = get_next_node(search_tree, leaf_nodes);
    end 
end