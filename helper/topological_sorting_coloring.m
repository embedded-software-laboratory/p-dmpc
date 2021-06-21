function [ topo_valid, topo_matrix ] = topological_sorting_coloring(coupling_undirected)
% TOPOLOGICAL_SORTING_COLORING  approximate best topological sorting
%                               regarding number of topological levels and incoming edges

    % apply topological sorting algorithm with coloring
    [topo_valid, topo_matrix] = topological_coloring(coupling_undirected);
    assert(topo_valid,'No valid topological coloring possible.');
    
    % determine the order to reduce incoming edges 
    order = order_topo(topo_matrix,coupling_undirected);
    
    % reorder matrix according to new level order
    topo_matrix(:,:) = topo_matrix(order,:);
end

