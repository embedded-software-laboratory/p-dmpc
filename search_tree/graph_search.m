function [u, y_pred, info] = graph_search(scenario, iter, prev_info)
    info = struct;
    trim_length = zeros(1, scenario.nVeh);
    for i = 1 : scenario.nVeh
        trim_length(i) = length(scenario.combined_graph.motionGraphList(i).trims);
    end
    
    % Create tree with root node
    % TODO Choose trim_indices depending on measurement
    cur_node_id = 1;
    init_depth = 0;
    g_values = zeros(scenario.nVeh,1);
    h_values = zeros(scenario.nVeh,1);
    cur_node = node(...
        cur_node_id, ...
        init_depth, ...
        prev_info.trim_indices, ...
        iter.x0(:,1), ...
        iter.x0(:,2), ...
        iter.x0(:,3), ...
        g_values, ...
        h_values...
    );
    info.tree = tree(cur_node);
    
    % Array storing ids of nodes that may be expanded
    max_id = 1;
    leaf_nodes = [];
    candidates = [];
    min_value = Inf;
    cur_value = sum(cur_node.g_values + cur_node.h_values);
    
    % Array storing ids of nodes that were visited
    visited_nodes = [];
    
    % Expand leaves of tree until depth or target is reached or until there 
    % are no leaves
    while ((cur_value < min_value && (cur_node.depth < h_p)) ...
           || isempty(candidates))
               
        % Delete chosen entry from list of expandable nodes

        % Expand chosen node
        [leaf_nodes, candidates, info.tree, max_id] = expand_node(scenario, iter, leaf_nodes, info.tree, cur_node_id, obstacles, ...
                                                              situation_costs, trim_length, visited_nodes, max_id);        
                                                                                   
        % visited_nodes = [visited_nodes, cur_node_id];  
        
        % TODO no difference between open nodes and candidates
        for candidate = candidates
            value = sum(info.tree.Node{candidate}.g_values + info.tree.Node{candidate}.h_values);
            if value < min_value
                min_value = value;
                final_node_id = candidate;
                y_pred = return_path_to(final_node_id, info.tree, scenario.combined_graph);
                % TODO u
                u = 0;
                p = pathtoroot(info.tree, final_node_id);
                info.trim_indices = info.tree.Node{p(end-1)}.trims;
            end
        end
        
        % get next node for expansion
        % TODO Sorted list of open nodes
        if ~isempty(leaf_nodes)
            cur_node_id = get_next_node(info.tree, leaf_nodes);
            cur_node = info.tree.Node{cur_node_id};   
            cur_value = sum(cur_node.g_values + cur_node.h_values);
            leaf_nodes(leaf_nodes == cur_node_id) = []; 
        else
            return
        end
        
    end 
end