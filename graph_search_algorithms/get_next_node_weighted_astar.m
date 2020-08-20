function node = get_next_node_weighted_astar(search_tree, leaf_nodes)

    % Advance tree expansion to next node by choosing closest leaf to
    % target 
    
    h_weight = 2;
    
    g_weight = 1;
    
    if (length(leaf_nodes) > 1)
        
        curr_node = search_tree.get(leaf_nodes(1));
            
        curr_values = g_weight*curr_node.gvalues + h_weight*curr_node.hvalues;
        min_value = sum(curr_values);
        
        node = leaf_nodes(1);

        for i = 2:length(leaf_nodes)
            
            if leaf_nodes(i) == []
            
                continue;
            
            end
            
            curr_node = search_tree.get(leaf_nodes(i));
            
            curr_values = g_weight*curr_node.gvalues + h_weight*curr_node.hvalues;
            curr_value = sum(curr_values);

            if min_value > curr_value
                min_value = curr_value;
                node = leaf_nodes(i);
            end

        end

    else
        
        node = leaf_nodes(1);
        
    end

end

