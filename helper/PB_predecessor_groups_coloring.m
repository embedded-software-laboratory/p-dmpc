function [ groups ] = PB_predecessor_groups_coloring(coupling_undirected)

    % apply kahns topological sorting algorithm
    [topo_valid, topo_matrix] = topological_coloring(coupling_undirected);
    assert(topo_valid,'No valid topological coloring possible.');
    
    % determine the order to reduce incoming edges 
    order = order_topo(topo_matrix,coupling_undirected);
    
    % reorder matrix according to new level order
    topo_matrix(:,:) = topo_matrix(order,:);

    % determine which vehicles can plan parallel
    groups = PB_predecessor_groups(topo_matrix);
    
end

