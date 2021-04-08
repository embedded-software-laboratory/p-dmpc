function [idx_tree, idx_nodes] = get_next_node(search_tree, open_nodes)
    % TODO delete function
    assert(~isempty(open_nodes));
    
    h_weight = 2;
    g_weight = 1;
    min_value = inf;
    
    for iNode = 1:length(open_nodes)
        values = g_weight * search_tree.Node{open_nodes(iNode)}.g_values ...
               + h_weight * search_tree.Node{open_nodes(iNode)}.h_values;
        cur_value = sum(values);
        if cur_value < min_value
            min_value = cur_value;
            idx_tree = open_nodes(iNode);
            idx_nodes = iNode;
        end
    end
end

