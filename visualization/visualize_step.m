function visualize_step(scenario, tree, parent, mpa)
    hold on
    title("Iteration: " + parent + ", Time: " + (parent * scenario.dt));
    n_veh = length(tree.node{1, 1}(:,NodeInfo.trim));
    parent_node = tree.node{parent};  
    if(parent > 1)
        pred = tree.Parent(parent);
        pred_node = tree.node{pred};   
        path_cell = path_between(pred_node, parent_node, tree, mpa);
        for iVeh = 1 : n_veh
            cur_color = vehColor(iVeh);
            path = path_cell{iVeh};
            if(~isempty(path))
                plot(path(:,1), path(:,2), '-','Color', cur_color, 'LineWidth', 2); 
                plot(parent_node(iVeh,NodeInfo.x), parent_node(iVeh,NodeInfo.y), 'o','Color', cur_color, 'MarkerSize',6,'MarkerFaceColor', cur_color);
                % text(parent_node(iVeh,NodeInfo.x) - 0.1, parent_node(iVeh,NodeInfo.y) - 0.1, num2str(parent), 'FontSize', 6);
            end
        end  
    else
        for iVeh = 1 : n_veh
            cur_color = vehColor(iVeh);
            plot(parent_node(iVeh,NodeInfo.x), parent_node(iVeh,NodeInfo.y), 'o','Color', cur_color, 'MarkerSize',6,'MarkerFaceColor', cur_color);
            % text(parent_node(iVeh,NodeInfo.x) - 0.1, parent_node(iVeh,NodeInfo.y) - 0.1, num2str(parent), 'FontSize', 6);
        end  
    end
    drawnow
    
end