function id = get_next_node(search_tree, leaf_nodes)    
    
    h_weight = 2;
    g_weight = 1;
    if (length(leaf_nodes) > 1)
        cur_node = search_tree.Node{leaf_nodes(1)};
        values = g_weight * cur_node.g_values + h_weight * cur_node.h_values;
        min_value = sum(values);
        id = leaf_nodes(1);
        
        for i = 2:length(leaf_nodes)
            cur_node = search_tree.Node{leaf_nodes(i)};
            values = g_weight * cur_node.g_values + h_weight * cur_node.h_values;
            cur_value = sum(values);
            
            if min_value > cur_value
                min_value = cur_value;
                id = leaf_nodes(i);
            end
        end
    else
        id = leaf_nodes(1);
    end
end

