function maneuver_matrix = compute_product_maneuver_matrix(trim_product,motion_graph_list)

    n_graphs = length(motion_graph_list);
    [n_trims,n_graphs2] = size(trim_product);
    
    % preallocate
    maneuver_matrix = zeros(n_trims,n_trims);
    
    % iterate through trim product list to get all combinations of trim products
    for i = 1 : n_trims
        
        start_trim_tuple = trim_product(i,:);
        
        for j = 1 : n_trims
            
            end_trim_tuple = trim_product(j,:);
            
            % check if every transition has a maneuver
            for k = 1 : n_graphs
            
                maneuvers = motion_graph_list(k).maneuvers;
                
                start_trim_index = start_trim_tuple(k);
                end_trim_index = end_trim_tuple(k);
                
                % lookup maneuver matrix if entry is empty
                if isequal(maneuvers{start_trim_index,end_trim_index} , [])
                    
                    maneuver_matrix(i,j) = 0;
                    break;
                
                else
                    
                    maneuver_matrix(i,j) = 1;
                    
                end
                
            end
            
        end
        
    end
 
end
