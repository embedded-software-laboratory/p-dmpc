function visited = is_visited(state,search_tree,visit_list,offset)

    nVeh = length(state.xs);
    nNodes = length(visit_list);

    visited = false;
    
    for i = 1 : nNodes
        
        visited = false;
    
        id = visit_list(i);
        
        comp_node = search_tree.get(id);
            
        trims = comp_node.trims;
        
        same_trims = state.trims == trims;
        
        if ~same_trims
           
            continue;
            
        end
        
        xs = comp_node.xs;
        ys = comp_node.ys;
        yaws = comp_node.yaws;
        
        for j = 1 : nVeh

            equal_pose = is_equal_pose(state.xs(j),state.ys(j),state.yaws(j),xs(j),ys(j),yaws(j),offset);
            
            if ~equal_pose
                
                visited = false;
                break;
                
            else
                
                visited = true;
                
            end
        
        end
        
        if visited
        
            return;
        
        end
    
    end

end

