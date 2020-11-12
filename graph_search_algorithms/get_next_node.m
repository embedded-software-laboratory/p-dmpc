function id = get_next_node(search_tree, leaf_nodes)    

    if (length(leaf_nodes) > 1)
        cur_node = search_tree.Node{leaf_nodes(1)};
        min_value = sum(cur_node.h_values);
        id = leaf_nodes(1);
        
        for i = 2:length(leaf_nodes)
            cur_node = search_tree.Node{leaf_nodes(i)};
            cur_value = sum(cur_node.h_values);
            
            if min_value > cur_value
                min_value = cur_value;
                id = leaf_nodes(i);
            end
        end
    else
        id = leaf_nodes(1);
    end
end

