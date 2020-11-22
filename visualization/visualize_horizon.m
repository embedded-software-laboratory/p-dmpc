function horizon = visualize_horizon(search_window, motion_graph, horizon, index)
    
    n_veh = length(motion_graph.motionGraphList);
    if index < 2
        node = search_window.Node{index};
        for i = 1:n_veh
            horizon(i) = plot(node.xs(i), node.ys(i), 'o','Color', 'black', 'MarkerSize',3,'MarkerFaceColor', 'black');
            horizon(i).Color(4) = 0.5;
        end
    else
        last_index = length(horizon(1).XData);
        for i = last_index:index
            node = search_window.Node{i};
            for j = 1:n_veh
                horizon(j).XData = [horizon(j).XData, node.xs(j)];
                horizon(j).YData = [horizon(j).YData, node.ys(j)];
            end
        end
    end
    drawnow
end