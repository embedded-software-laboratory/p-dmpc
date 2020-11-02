function [search_tree, leaf_nodes] = generate_horizon(init_poses, target_poses, trim_indices, combined_graph)

    n_veh = length(combined_graph.motionGraphList);
    trim_length = zeros(1, n_veh);
    for i = 1 : n_veh
        trim_length(i) = length(combined_graph.motionGraphList(i).trims);
    end

    % Initialize node table values
    g_values = zeros(1,n_veh);
    hvalues = Inf(1,n_veh);
    trims = trim_indices;
    xs = [init_poses(1:n_veh).x];
    ys = [init_poses(1:n_veh).y];
    yaws = [init_poses(1:n_veh).yaw];
    depth = 0;
    
    % Create tree with root node
    node1 = node(depth, trims, xs, ys, yaws, g_values, hvalues);
    search_tree = tree(node1);
    
    % Array storing ids of nodes that may be expanded
    id = 1;
    leaf_nodes = [id];
    next_node_id = id;
    
    % Array storing ids of nodes that were visited
    visited_nodes = [];
    
    % Initialize 
    offset = ones(1, n_veh);
    is_goals = is_goal(init_poses, target_poses);
    
    % Expand leaves of tree until depth or target is reached or until there 
    % are no leaves
    while (search_tree.Node{id}.depth < h_u) && ~(sum(is_goals) == n_veh) && ~isempty(leaf_nodes)
               
        % Delete chosen entry from list of expandable nodes
        leaf_nodes(leaf_nodes == next_node_id) = [];
        [leaf_nodes, search_tree, id, is_goals] = expand_tree(leaf_nodes, search_tree, next_node_id, combined_graph, ...
                                                              trim_length, target_poses, visited_nodes, id, is_goals, ...
                                                              @is_collision);
        
        visited_nodes = [visited_nodes, next_node_id];       
        
        % get next node for expansion
        next_node_id = get_next_node_weighted_astar(search_tree, leaf_nodes);
    end 
    
    if ~(sum(is_goals) == n_veh)
        [search_tree, leaf_nodes] = extend_horizon(leaf_nodes, search_tree, combined_graph, target_poses);
    end
end