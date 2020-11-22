function [search_window, leaf_nodes] = generate_horizon(init_poses, target_poses, cur_node, trim_indices, motion_graph, video)

    n_veh = length(motion_graph.motionGraphList);
    trim_length = zeros(1, n_veh);
    for i = 1 : n_veh
        trim_length(i) = length(motion_graph.motionGraphList(i).trims);
    end
    
    % Reset horizon plot 
    horizon = gobjects(1, n_veh);
    
    % Create tree with root node
    search_window = tree(node(0, trim_indices, cur_node.xs, cur_node.ys, cur_node.yaws, cur_node.g_values, cur_node.h_values));
    
    % Array storing ids of nodes that may be expanded
    id = 1;
    leaf_nodes = [id];
    next_node_id = id;
    
    % Initial horizon
    horizon = visualize_horizon(search_window, motion_graph, horizon, id);
    frame = getframe(gcf);
    writeVideo(video, frame);
    
    % Array storing ids of nodes that were visited
    visited_nodes = [];
    
    % Initialize
    cur_poses = node2poses(cur_node);
    is_goals = is_goal(cur_poses, target_poses);
    
    % Expand leaves of tree until depth or target is reached or until there 
    % are no leaves
    while (((search_window.Node{id}.depth < h_u) && ~(sum(is_goals) == n_veh) && ~isempty(leaf_nodes))...
            || (cheaper(search_window.Node{next_node_id}, search_window.Node{id}) && (search_window.Node{next_node_id}.depth < h_u)))
               
        % Delete chosen entry from list of expandable nodes
        leaf_nodes(leaf_nodes == next_node_id) = [];
        [leaf_nodes, search_window, id, is_goals] = expand_horizon(leaf_nodes, search_window, next_node_id, motion_graph, ...
                                                              trim_length, init_poses, target_poses, visited_nodes, id, is_goals);
        
        visited_nodes = [visited_nodes, next_node_id];    
        horizon = visualize_horizon(search_window, motion_graph, horizon, id);
        frame = getframe(gcf);
        writeVideo(video, frame);
        
        % get next node for expansion
        next_node_id = get_next_node(search_window, leaf_nodes);
    end 
    
    if ~(sum(is_goals) == n_veh)
        delete(horizon);
    end
end