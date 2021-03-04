function [video, search_tree] = receding_horizon(init_poses, target_poses, trim_indices, obstacles, combined_graph, situation_costs, video)
%RECEDING_HORIZON Explore path to target using a receding horizon 

    % Initialize
    n_veh = length(combined_graph.motionGraphList);
    horizon = cell(1, 3);
    p = gobjects(1, n_veh);
    g = gobjects(1, n_veh);
    for i = 1:n_veh
        cur_color = vehColor(i);
        p(i) = plot(init_poses.xs(i), init_poses.ys(i), '-','Color', cur_color, 'LineWidth', 2);
        p(i).Color(4) = 0.5;
        g(i) = plot(init_poses.xs(i), init_poses.ys(i), 'o','Color', cur_color, 'MarkerSize',3,'MarkerFaceColor', cur_color);
        g(i).Color(4) = 0.5;
    end
    cur_depth = 1;
    cur_node = node(1, 0, trim_indices, init_poses.xs, init_poses.ys, init_poses.yaws, zeros(1,n_veh), zeros(1,n_veh));
    search_tree = tree(cur_node);
    trims = trim_indices;

    % Check if the vehicle reached the destination
    is_goals = is_goal(init_poses, target_poses);
    while true
               
        % Continue receding horizon search
        [search_window, leaf_nodes, final_node_id, horizon] = generate_horizon(init_poses, target_poses, cur_node, trims, obstacles, combined_graph, situation_costs, horizon, video);
              
        if ~isnan(final_node_id)
            node_id = final_node_id;
        else
            visualize_trajectory(prev_search_window, prev_node_id, combined_graph);
            return
        end
        
        % Visualize horizon
        path_cell = return_path_to(node_id, search_window, combined_graph);
        for i = 1:n_veh
            path = path_cell{i};
            if ~isempty(path)
                p(i).XData = path(:,1);
                p(i).YData = path(:,2);
            end
        end
        drawnow;
        
        
        % Visualize path before addition to properly display horizon
        visualize_step(search_tree, cur_depth, combined_graph);
        frame = getframe(gcf);
        writeVideo(video, frame);

        % Determine next node
        search_path = findpath(search_window, 1, node_id);
        if length(search_path) > 1
            next_node_id = search_path(2);
        else
            return
        end
        cur_node = search_window.Node{next_node_id};
        trims = cur_node.trims;

        % Add node to tree
        [search_tree, cur_depth] = search_tree.addnode(cur_depth, cur_node);

        % Check if we already reached our destination
        is_goals = is_goal(cur_node, target_poses);
        if(sum(is_goals) == n_veh)
            visualize_step(search_tree, cur_depth, combined_graph);
            frame = getframe(gcf);
            writeVideo(video, frame);
            return
        end
        
        prev_search_window = search_window;
        prev_node_id = node_id;
        
    end
end