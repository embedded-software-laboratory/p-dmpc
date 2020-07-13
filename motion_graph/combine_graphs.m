function combinedGraph = combine_graphs(motionGraphList)

    % compute trim tuple (vertices)
    trimProduct = compute_trim_product(motionGraphList);
    
    %nTrimProd = length(trimProduct);
            
    % compute maneuver matrix for trimProduct
    maneuverMatrix = compute_product_maneuver_matrix(trimProduct, motionGraphList);
    
    combinedGraph.trimTuple = trimProduct;
    combinedGraph.transitionMatrix = maneuverMatrix;
  
end
