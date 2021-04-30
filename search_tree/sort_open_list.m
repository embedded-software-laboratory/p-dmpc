function [open_nodes, open_values] = sort_open_list(open_nodes, open_values)
    [open_values, indices] = sort(open_values);
    open_nodes = open_nodes(indices);
end