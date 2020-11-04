function visited = has_visited(index, state,search_tree, visit_list, tolerance)

    n_nodes = length(visit_list);
    visited = false;
    
    for i = 1 : n_nodes
        
        id = visit_list(i);
        comp_node = search_tree.get(id);
        trims = comp_node.trims;
        same_trims = isequal(state.trims,trims);
        
        if ~same_trims
            continue;
        end
        
        xs = comp_node.xs;
        ys = comp_node.ys;
        yaws = comp_node.yaws;
        
        equal_pose = is_equal_pose(state.xs(index),state.ys(index),state.yaws(index),xs(index),ys(index),yaws(index), tolerance);

        if ~equal_pose
            visited = false;
            break;
        else
            visited = true;
        end
        
        if visited
            return;
        end
    end
end

