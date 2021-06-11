function horizon = visualize_horizon(search_window, mpa, horizon)
    
    n_veh = length(mpa.trims);
    
    node = search_window.node{1};
    for i = 1:n_veh
        horizon{1}(i) = plot(node(iVeh,NodeInfo.x), node(iVeh,NodeInfo.y), 'o', 'Color', 'black', 'MarkerSize', 3, 'MarkerFaceColor', 'black');
        horizon{1}(i).Color(4) = 0.5;
    end

    last_index = length(horizon{1}(1).XData);
    max_index = length(search_window.node);
    for i = last_index:max_index
        node = search_window.node{i};
        for j = 1:n_veh
            if ~any(ismember(horizon{1}(j).XData, node.xs(j))) || ~any(ismember(horizon{1}(j).YData, node.ys(j)))
                horizon{2}(i, j) = text(node.xs(j)+0.1, node.ys(j)-0.1, num2str(i), 'FontSize', 6);
                horizon{1}(j).XData = [horizon{1}(j).XData, node.xs(j)];
                horizon{1}(j).YData = [horizon{1}(j).YData, node.ys(j)];
            end
        end
    end
    drawnow
end