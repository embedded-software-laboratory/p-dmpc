function [video, search_tree] = receding_horizon(init_poses, target_poses, trim_indices, combined_graph, video)
%RECEDING_HORIZON Explore path to target using a receding horizon 

    % Initialize
    n_veh = length(combined_graph.motionGraphList);
    horizon = cell(1, 3);
    p = gobjects(1, n_veh);
    g = gobjects(1, n_veh);
    for i = 1:n_veh
        cur_color = vehColor(i);
        p(i) = plot(init_poses.xs(i), init_poses.ys(i), '-','Color', cur_color);
        p(i).Color(4) = 0.5;
        g(i) = plot(init_poses.xs(i), init_poses.ys(i), 'o','Color', cur_color, 'MarkerSize',3,'MarkerFaceColor', cur_color);
        g(i).Color(4) = 0.5;
    end
    cur_depth = 1;
    cur_node = node(0, trim_indices, init_poses.xs, init_poses.ys, init_poses.yaws, zeros(1,n_veh), zeros(1,n_veh));
    search_tree = tree(cur_node);
    trims = trim_indices;

    % Check if the vehicle reached the destination
    is_goals = is_goal(init_poses, target_poses);
    while(sum(is_goals) ~= n_veh)
               
        % Continue receding horizon search
        [search_window, leaf_nodes, final_nodes, horizon] = generate_horizon(init_poses, target_poses, cur_node, trims, combined_graph, horizon, video);
              
        % Check if we already reached our destination
        last_id = length(search_window.Node);
        final_node = search_window.Node{end};
        is_goals = is_goal(final_node, target_poses);
        if(sum(is_goals) == n_veh)
            search_path = findpath(search_window, 1, length(search_window.Node));
            length_path = length(search_path);
            for i = 1:length_path
                [search_tree, cur_depth] = search_tree.addnode(cur_depth, search_window.Node{search_path(i)});

                % Visualize
                path_cell = return_path(search_window, combined_graph);
                for j = 1:n_veh
                    path = path_cell{j};
                    if ~isempty(path)
                        p(j).XData = path(:,1);
                        p(j).YData = path(:,2);
                    end
                end
                drawnow;
                visualize_step(search_tree, cur_depth, combined_graph);
                frame = getframe(gcf);
                writeVideo(video, frame);
            end
            return
        end
        
        if(~isempty(final_nodes))
            node_id = get_next_node(search_window, final_nodes);
        else
            node_id = last_id;
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
        next_node_id = search_path(2);
        cur_node = search_window.Node{next_node_id};
        trims = cur_node.trims;

        % Add node to tree
        [search_tree, cur_depth] = search_tree.addnode(cur_depth, cur_node);

        % Update our loop condition
        is_goals = is_goal(cur_node, target_poses);
    end
end