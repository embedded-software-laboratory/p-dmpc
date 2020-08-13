function visited = is_visited(state,search_tree,visit_list,offset)

    nVeh = length(state.xs);
    nNodes = length(visit_list);
    
    visited = false;
    
    for i = 1 : nNodes
    
        id = visit_list(i);
            
        trims = search_tree.get(id).trims;
        
        same_trims = isequal(state.trims, trims);
        
        if ~same_trims
           
            continue;
            
        end
        
        values = search_tree.get(id).values;
        xs = search_tree.get(id).xs;
        ys = search_tree.get(id).ys;
        yaws = search_tree.get(id).yaws;
        driven = search_tree.get(id).driven;
        
        for j = 1 : nVeh

            equal_pose = is_equal_pose(state.xs(j),state.ys(j),state.yaws(j),xs(j),ys(j),yaws(j),offset);
            
            if same_trims && equal_pose
                
                visited = true;
                return;
                
            end
        
        end
    
    end

end

