function productTrims = compute_trim_product(motionGraphList)

    nGraphs = length(motionGraphList);
    
    % iterate through all motionGraphs
    for i = 1 : nGraphs
        
        % save trim indices (1 to n_trims)
        trimIndexList{i} = [1:length(motionGraphList(i).trims)];
    
    end
    
    % cartesian product for all combinations of trims;
    trimProd = cartprod(trimIndexList{:});
    
    productTrims = trimProd;

end
