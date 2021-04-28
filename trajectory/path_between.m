function search_path = path_between(cur_node,next_node, search_tree, motion_graph)
%PATH_BETWEEN Return path as a cell array between two nodes

    n_veh = length(search_tree.Node{1, 1}.trims);
    search_path = cell(1, n_veh);
    for i = 1:n_veh        
        if(cur_node.xs(i) ~= next_node.xs(i) || cur_node.ys(i) ~= next_node.ys(i))
            maneuver = motion_graph.motionGraphList(i).maneuvers{cur_node.trims(i), next_node.trims(i)};
            assert(~isempty(maneuver),'manuevers{%d, %d} is empty.',cur_node.trims(i), next_node.trims(i));
            
            x = cur_node.xs(i);
            y = cur_node.ys(i);
            yaw = cur_node.yaws(i);
            
            xs = maneuver.xs;
            ys = maneuver.ys;
            yaws = maneuver.yaws + yaw;       
            
            length_maneuver = length(xs);
            trims = cur_node.trims(i) * ones(length_maneuver, 1);
            
            [xs, ys] = translate_global(yaw, x, y, xs, ys);
            
            search_path(i) = {[xs', ys', yaws', trims]};
        end
    end
end

