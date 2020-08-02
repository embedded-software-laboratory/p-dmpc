function visited = is_visited(state,search_graph,visit_list,offset)

    nVeh = length(state.xs);
    nNodes = length(visit_list);
    
    for i = 1 : nNodes
    
        node_id = visit_list(i);
        
        trims = search_graph.Nodes{node_id, 3};
        xs = search_graph.Nodes{node_id, 4};
        ys = search_graph.Nodes{node_id, 5};
        yaws = search_graph.Nodes{node_id, 6};
        
        same_trims = (state.trims == trims);
        
        for j = 1 : nVeh

            equal_pose = is_equal_pose(state.xs(j),state.ys(j),state.yaws(j),xs(j),ys(j),yaws(j),offset);
            
            if same_trims && equal_pose
                
                visited = true;
                return;
                
            end
        
        end
    
    end

end

