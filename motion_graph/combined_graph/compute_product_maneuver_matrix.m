function maneuverMatrix = compute_product_maneuver_matrix(trimProduct,motionGraphList)

    nGraphs = length(motionGraphList);
    [nTrims,nGraphs2] = size(trimProduct);
    
    % preallocate
    maneuverMatrix = zeros(nTrims,nTrims);
    
    % iterate through trim product list to get all combinations of trim products
    for i = 1 : nTrims
        
        startTrimTuple = trimProduct(i,:);
        
        for j = 1 : nTrims
            
            endTrimTuple = trimProduct(j,:);
            
            % check if every transition has a maneuver
            for k = 1 : nGraphs
            
                maneuvers = motionGraphList(k).maneuvers;
                
                startTrimIndex = startTrimTuple(k);
                endTrimIndex = endTrimTuple(k);
                
                % lookup maneuver matrix if entry is empty
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
