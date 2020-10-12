function visualize_step(search_tree, parent, motion_graph)

    pbaspect([1 1 1]);
    n_veh = length(search_tree.Node{1, 1}.trims);
    vehColors = [0.8941    0.1020    0.1098;...
                 0.2157    0.4941    0.7216;...
                 0.3020    0.6863    0.2902;...
                 0.5961    0.3059    0.6392;...
                 1.0000    0.4980    0     ;...
                 1.0000    1.0000    0.2000;...
                 0.6510    0.3373    0.1569;...
                 0.9686    0.5059    0.7490];
    
    hold on
    parent_node = search_tree.Node{parent};  
    pred = search_tree.Parent(parent);
    pred_node = search_tree.Node{pred};   
    path_cell = path_between(pred_node, parent_node, search_tree, motion_graph);
    for j = 1 : n_veh
        cur_color = vehColors(mod(j-1,size(vehColors,1))+1,:);
        path = path_cell{j};
        plot(path(:,1), path(:,2), '--','Color', cur_color);
        plot(parent_node.xs(j), parent_node.ys(j), 'o','Color', cur_color);
    end   
    pause(0.2);
end