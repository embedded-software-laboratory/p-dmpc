function open_values = sum_values(tree,open_nodes)
% SUM_VALUES    Sum values for multiple vehicles.
    h_weight = 1;
    g_weight = 1;
    open_values = tree.g(open_nodes) * g_weight + tree.h(open_nodes) * h_weight;
end
