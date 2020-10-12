function visualize_trajectory(search_tree, parents, motion_graph)
   
    paths_cell = return_path(search_tree, motion_graph);
    pbaspect([1 1 1]);
    n_veh = length(paths_cell);
    vehColors = [0.8941    0.1020    0.1098;...
                 0.2157    0.4941    0.7216;...
                 0.3020    0.6863    0.2902;...
                 0.5961    0.3059    0.6392;...
                 1.0000    0.4980    0     ;...
                 1.0000    1.0000    0.2000;...
                 0.6510    0.3373    0.1569;...
                 0.9686    0.5059    0.7490];
    plots = gobjects(1, 2*n_veh);
    descriptions = strings(1, 2*n_veh); 
    
    length_parents = length(parents);
    for i = 1 : length_parents
        hold on
        parent_node = search_tree.Node{parents(i)};  
        if(i > 1)
            pred = search_tree.Parent(parents(i));
            pred_node = search_tree.Node{pred};   
            path_cell = path_between(pred_node, parent_node, search_tree, motion_graph);
            for j = 1 : n_veh
                cur_color = vehColors(mod(j-1,size(vehColors,1))+1,:);
                path = path_cell{j};
                if(ismember(path,paths_cell{j}))
                    plots(j) = plot(path(:,1), path(:,2), '--','Color', cur_color);
                    descriptions(j) = 'Trajectory Vehicle: ' + string(j);
                end
                plots(j + n_veh) = plot(parent_node.xs(j), parent_node.ys(j), 'o','Color', cur_color);
                descriptions(j + n_veh) = 'Explored Nodes Vehicle: ' + string(j);
            end
        else
            for j = 1 : n_veh
                cur_color = vehColors(mod(j-1,size(vehColors,1))+1,:);
                plots(j + n_veh) = plot(parent_node.xs(j), parent_node.ys(j), 'o','Color', cur_color);
                descriptions(j + n_veh) = 'Explored Nodes Vehicle: ' + string(j);
            end
        end
        pause(0.2);
    end
    
    end_node = search_tree.Node{end};
    pred = search_tree.Parent(end);
    pred_node = search_tree.get(pred);
    path_cell = path_between(pred_node, end_node, search_tree, motion_graph);
    for i = 1 : n_veh
        hold on
        cur_color = vehColors(mod(i-1,size(vehColors,1))+1,:);
        path = path_cell{i};
        plot(path(:,1), path(:,2), '--','Color', cur_color);
        plot(end_node.xs(i), end_node.ys(i), 'o','Color', cur_color);
    end
        
end