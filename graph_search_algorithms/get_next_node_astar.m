function node = get_next_node_astar(search_tree, leaf_nodes)

    % Advance tree expansion to next node by choosing closest leaf to
    % target 
    
    if (length(leaf_nodes) > 1)

        curr_values = search_tree.get(leaf_nodes(1)).values;
        min_value = sum(curr_values);
        node = leaf_nodes(1);

        for i = 2:length(leaf_nodes)
            
            curr_values = search_tree.get(leaf_nodes(i)).values;
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

