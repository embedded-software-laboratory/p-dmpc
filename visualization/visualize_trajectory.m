function visualize_trajectory(search_tree, dest_id, motion_graph)
    path = findpath(search_tree, 1, dest_id);
    
    for node = path
        visualize_step(scenario, search_tree, node, motion_graph);
    end
end