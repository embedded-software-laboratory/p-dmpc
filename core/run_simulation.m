function scenario = run_simulation(options)
    scenario = Scenario(options.angles);
    [init_poses, target_poses] = create_poses(scenario);
    
    depth = 3;
    trim_indices = 4 * ones(1, options.amount);

    % make motionGraph Tupel
    motionGraphList = create_motion_graph_list('trim_set_4_1', options.amount);

    % Set figure
    figure('units','normalized','outerposition',[0.125 0.125 0.75 0.75]);
    pbaspect([1 1 1]);
    axis([-35 35 -35 35]);
    title("Iteration: 0");
    draw_destination(target_poses);
    draw_cars(init_poses);
    
    % Combine graphs
    combined_graph = CombinedGraph(motionGraphList);
    [search_tree] = receding_horizon(init_poses, target_poses, trim_indices, combined_graph, depth, @is_collision, @get_next_node_weighted_astar);
end

