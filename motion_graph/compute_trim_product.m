function productTrims = compute_trim_product(motionGraphList)

    nGraphs = length(motionGraphList);
    
    %% trim product as index tuple
    maxTrims = 0;
    % find max length of trims
    for i = 1 : nGraphs
        
        % find maximal length of trims for preallocating
        if length(motionGraphList(i).trims) > maxTrims
            
            maxTrims = length(motionGraphList(i).trims)
            
        end
    
    end
    
        
    % preallocate
    trimProd = ones(maxTrims,nGraphs);          % for testing, should be zeros(...)

    % fill "tuple" matrix
    % --- TODO: Schleifenschachtelungs-Tiefe unbekannt (?) ---
    for i = 1 : nGraphs
        
        for j = 1 : length(motionGraphList(i).trims)
        
            % --- TODO ---
            
        end
    
    end

    %% return
    productTrims = trimProd;

end
