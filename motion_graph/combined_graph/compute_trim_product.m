function product_trims = compute_trim_product(motion_graph_list)

    n_graphs = length(motion_graph_list);
    
    % iterate through all motion_graphs
    for i = 1 : n_graphs
        
        % save trim indices (1 to n_trims)
        trim_index_list{i} = [1:length(motion_graph_list(i).trims)];
    
    end
    
    % cartesian product for all combinations of trims;
    trim_prod = cartprod(trim_index_list{:});
    
    product_trims = trim_prod;

end
