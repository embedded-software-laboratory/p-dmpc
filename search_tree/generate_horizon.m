function [search_window, leaf_nodes, final_node_id, horizon] = generate_horizon(iter, init_poses, target_poses, init_node, trim_indices, obstacles, motion_graph, situation_costs, horizon, video)

    n_veh = length(motion_graph.motionGraphList);
    trim_length = zeros(1, n_veh);
    for i = 1 : n_veh
        trim_length(i) = length(motion_graph.motionGraphList(i).trims);
    end
    
    % Create tree with root node
    search_window = tree(node(1, 0, trim_indices, init_node.xs, init_node.ys, init_node.yaws, init_node.g_values, init_node.h_values));
    
    % Array storing ids of nodes that may be expanded
    max_id = 1;
    leaf_nodes = [];
    cur_node_id = 1;
    cur_node = search_window.Node{cur_node_id};
    candidates = [];
    final_node_id = NaN;
    min_value = Inf;
    cur_value = sum(cur_node.g_values + cur_node.h_values);
    
    % Array storing ids of nodes that were visited
    visited_nodes = [];
    
    % Expand leaves of tree until depth or target is reached or until there 
    % are no leaves
    while ((cur_value < min_value && (cur_node.depth < h_p)) ...
           || isempty(candidates))
               
        % Delete chosen entry from list of expandable nodes
        [leaf_nodes, candidates, search_window, max_id] = expand_horizon(iter, leaf_nodes, search_window, cur_node_id, obstacles, motion_graph, ...
                                                              situation_costs, trim_length, init_poses, target_poses, visited_nodes, max_id);        
                                                                                   
        visited_nodes = [visited_nodes, cur_node_id];  
        
        for candidate = candidates
            value = sum(search_window.Node{candidate}.g_values + search_window.Node{candidate}.h_values);
            if value < min_value
                min_value = value;
                final_node_id = candidate;
            end
        end
        
        % get next node for expansion
        if ~isempty(leaf_nodes)
            cur_node_id = get_next_node(search_window, leaf_nodes);
            cur_node = search_window.Node{cur_node_id};   
            cur_value = sum(cur_node.g_values + cur_node.h_values);
            leaf_nodes(leaf_nodes == cur_node_id) = []; 
        else
            return
        end
        
    end 
end