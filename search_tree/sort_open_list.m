function [open_nodes_sorted, open_values_sorted] = sort_open_list(open_nodes, open_values)
    [open_values_sorted, indices] = sort(open_values);
    open_nodes_sorted = open_nodes(indices);
end