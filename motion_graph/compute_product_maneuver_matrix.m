function maneuverMatrix = compute_product_maneuver_matrix(trimProduct,motionGraphList)

    nGraphs = length(motionGraphList);
    [nTrims,nGraphs2] = size(trimProduct);
    
    % preallocate
    maneuverMatrix = zeros(nTrims,nTrims);
    
    for i = 1 : nTrims
        
        startTrimTuple = trimProduct(i,:);
        
        for j = 1 : nTrims
            
            endTrimTuple = trimProduct(j,:);
            
            for k = 1 : nGraphs
            
                maneuvers = motionGraphList(k).maneuvers;
                
                startTrimIndex = startTrimTuple(k);
                endTrimIndex = endTrimTuple(k);
                
                if isequal(maneuvers{startTrimIndex,endTrimIndex} , [])
                    
                    maneuverMatrix(i,j) = 0;
                    break;
                
                else
                    
                    maneuverMatrix(i,j) = 1;
                    
                end
                
            end
            
        end
        
    end
 
end
