function open_values = sum_values(search_tree,open_nodes)
    h_weight = 2;
    g_weight = 1;
    open_values = zeros(size(open_nodes));
    for iNode = 1:length(open_nodes)
        values = g_weight * search_tree.Node{open_nodes(iNode)}(:,search_tree.idx.g) ...
               + h_weight * search_tree.Node{open_nodes(iNode)}(:,search_tree.idx.h);
        open_values(iNode) = sum(values);
    end
end