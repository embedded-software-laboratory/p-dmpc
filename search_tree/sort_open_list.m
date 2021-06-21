function [open_nodes, open_values] = sort_open_list(open_nodes, open_values)
% SORT_OPEN_LIST    Sort values and nodes according to that order.

    [open_values, indices] = sort(open_values);
    open_nodes = open_nodes(indices);
end