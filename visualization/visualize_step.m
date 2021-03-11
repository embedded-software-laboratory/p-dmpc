function visualize_step(search_tree, parent, motion_graph)

    hold on
    title("Iteration: " + parent + ", Time: " + (parent * dt));
    n_veh = length(search_tree.Node{1, 1}.trims);
    parent_node = search_tree.Node{parent};  
    if(parent > 1)
        pred = search_tree.Parent(parent);
        pred_node = search_tree.Node{pred};   
        path_cell = path_between(pred_node, parent_node, search_tree, motion_graph);
        for j = 1 : n_veh
            cur_color = vehColor(j);
            path = path_cell{j};
            if(~isempty(path))
                plot(path(:,1), path(:,2), '-','Color', cur_color, 'LineWidth', 2); 
                plot(parent_node.xs(j), parent_node.ys(j), 'o','Color', cur_color, 'MarkerSize',6,'MarkerFaceColor', cur_color);
                % text(parent_node.xs(j) - 0.1, parent_node.ys(j) - 0.1, num2str(parent), 'FontSize', 6);
            end
        end  
    else
        for j = 1 : n_veh
            cur_color = vehColor(j);
            plot(parent_node.xs(j), parent_node.ys(j), 'o','Color', cur_color, 'MarkerSize',6,'MarkerFaceColor', cur_color);
            % text(parent_node.xs(j) - 0.1, parent_node.ys(j) - 0.1, num2str(parent), 'FontSize', 6);
        end  
    end
    drawnow
    
end