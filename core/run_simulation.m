function scenario = run_simulation(angles)
    scenario = Scenario(angles);
    [init_poses, target_poses] = create_poses(scenario);
    
    depth = 2;

    % make motionGraph Tupel
    motionGraphList = create_motion_graph_list('trim_set_6_1', 2);

    % Set figure
    axis_size = [-10 20 -10 20];
    figure('units','normalized','outerposition',[0.125 0.125 0.75 0.75]);

    % Combine graphs
    combined_graph = CombinedGraph(motionGraphList);
    [search_tree] = receding_horizon(init_poses, target_poses, [5,5], combined_graph, 2, @is_collision, @get_next_node_weighted_astar);
end

