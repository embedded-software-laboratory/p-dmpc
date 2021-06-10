function visualize_trajectory(tree, dest_id, mpa)
    path = findpath(tree, 1, dest_id);
    
    for node = path
        visualize_step(scenario, tree, node, mpa);
    end
end