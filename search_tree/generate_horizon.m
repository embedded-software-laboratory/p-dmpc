function [search_window, leaf_nodes, final_nodes, horizon, is_goals] = generate_horizon(init_poses, target_poses, cur_node, trim_indices, motion_graph, horizon, video)

    n_veh = length(motion_graph.motionGraphList);
    trim_length = zeros(1, n_veh);
    for i = 1 : n_veh
        trim_length(i) = length(motion_graph.motionGraphList(i).trims);
    end
    
    % Create tree with root node
    search_window = tree(node(0, trim_indices, cur_node.xs, cur_node.ys, cur_node.yaws, cur_node.g_values, cur_node.h_values));
    
    % Array storing ids of nodes that may be expanded
    max_id = 1;
    leaf_nodes = [max_id];
    final_nodes = [];
    next_node_id = max_id;
    min_value = Inf;
    cur_value = Inf;
    candidate_found = false;
    
    % Array storing ids of nodes that were visited
    visited_nodes = [];
    
    % Initialize
    is_goals = is_goal(cur_node, target_poses);
    
    % Expand leaves of tree until depth or target is reached or until there 
    % are no leaves
    while (~isempty(leaf_nodes) ...
           && (~candidate_found || (next_value < min_value && (next_node.depth < h_p))) ...
           && ~(sum(is_goals) == n_veh))
               
        % get next node for expansion
        next_node_id = get_next_node(search_window, leaf_nodes);
        leaf_nodes(leaf_nodes == next_node_id) = [];   
        
        % Delete chosen entry from list of expandable nodes
        [leaf_nodes, final_nodes, search_window, max_id, is_goals] = expand_horizon(leaf_nodes, final_nodes, search_window, next_node_id, motion_graph, ...
                                                              trim_length, init_poses, target_poses, visited_nodes, max_id, is_goals);        
                                                                                   
        visited_nodes = [visited_nodes, next_node_id];  
        
        next_node = search_window.Node{next_node_id};   
        next_value = sum(next_node.g_values + next_node.h_values);
        if next_value < min_value
            min_value = next_value;
        end

        if ~candidate_found && search_window.Node{max_id}.depth == h_p
            candidate_found = true;
        end
    end 
      
    % Reset horizon plot 
    delete(horizon{1});
    delete(horizon{2});
    horizon{1} = gobjects(1, n_veh);
    horizon{2} = [gobjects(1, n_veh)];
    horizon = visualize_horizon(search_window, motion_graph, horizon);
    frame = getframe(gcf);
    writeVideo(video, frame);
end