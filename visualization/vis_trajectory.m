function vis_trajectory(search_tree, parents, motion_graph, target_poses, axis_size)

    paths = return_path(search_tree, motion_graph);
    figure('units','normalized','outerposition',[0.125 0.125 0.75 0.75])
    pbaspect([1 1 1]);
    n_veh = length(paths(1,1,:));
    height = length(paths(:,1,1));
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
    axis(axis_size);
                        
    for i = 1:n_veh
        hold on
        cur_color = vehColors(mod(i-1,size(vehColors,1))+1,:);
        radius = 1;
        center = [target_poses(i).x, target_poses(i).y];
        position = [center - radius, 2*radius, 2*radius];
        rectangle('Position',position,'Curvature',[1 1], 'EdgeColor', cur_color);
        text(center(1)+radius,center(2)+0.5*radius,'DestVeh:' + string(i));
    end
    
    drawnow;
    length_parents = length(parents);
        
    for i = 1 : length_parents
        hold on
        parent_node = search_tree.get(parents(i));       
        for j = 1 : n_veh
            cur_color = vehColors(mod(j-1,size(vehColors,1))+1,:);
            if(i > 1)
                pred = parent_node.parent;
                pred_node = search_tree.get(pred);            
                path = path_between(pred_node, parent_node, search_tree, motion_graph);
                if(ismember(path,paths))
                    plots(j) = plot(path(:,1,j), path(:,2,j), '--','Color', cur_color);
                    descriptions(j) = 'Trajectory Vehicle: ' + string(j);
                end
            end
            plots(j + n_veh) = plot(parent_node.xs(j), parent_node.ys(j), 'o','Color', cur_color);
            descriptions(j + n_veh) = 'Explored Nodes Vehicle: ' + string(j);
        end
        pause(0.2);
    end
    
    end_node = search_tree.Node{end};
    for i = 1 : n_veh
        hold on
        cur_color = vehColors(mod(i-1,size(vehColors,1))+1,:);
        pred = end_node.parent;
        pred_node = search_tree.get(pred);
        path = path_between(pred_node, end_node, search_tree, motion_graph);
        plot(path(:,1,i), path(:,2,i), '--','Color', cur_color);
        plot(end_node.xs(i), end_node.ys(i), 'o','Color', cur_color);
    end
    
    legend(plots, descriptions);
    
end