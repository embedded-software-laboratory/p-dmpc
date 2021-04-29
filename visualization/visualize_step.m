function visualize_step(scenario, search_tree, parent, motion_graph)
    hold on
    title("Iteration: " + parent + ", Time: " + (parent * scenario.dt));
    n_veh = length(search_tree.Node{1, 1}(:,search_tree.idx.trim));
    parent_node = search_tree.Node{parent};  
    if(parent > 1)
        pred = search_tree.Parent(parent);
        pred_node = search_tree.Node{pred};   
        path_cell = path_between(pred_node, parent_node, search_tree, motion_graph);
        for iVeh = 1 : n_veh
            cur_color = vehColor(iVeh);
            path = path_cell{iVeh};
            if(~isempty(path))
                plot(path(:,1), path(:,2), '-','Color', cur_color, 'LineWidth', 2); 
                plot(parent_node(iVeh,search_tree.idx.x), parent_node(iVeh,search_tree.idx.y), 'o','Color', cur_color, 'MarkerSize',6,'MarkerFaceColor', cur_color);
                % text(parent_node(iVeh,search_tree.idx.x) - 0.1, parent_node(iVeh,search_tree.idx.y) - 0.1, num2str(parent), 'FontSize', 6);
            end
        end  
    else
        for iVeh = 1 : n_veh
            cur_color = vehColor(iVeh);
            plot(parent_node(iVeh,search_tree.idx.x), parent_node(iVeh,search_tree.idx.y), 'o','Color', cur_color, 'MarkerSize',6,'MarkerFaceColor', cur_color);
            % text(parent_node(iVeh,search_tree.idx.x) - 0.1, parent_node(iVeh,search_tree.idx.y) - 0.1, num2str(parent), 'FontSize', 6);
        end  
    end
    drawnow
    
end