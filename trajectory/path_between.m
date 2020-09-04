function search_paths = path_between(cur_node,next_node, search_tree, motion_graph)
%PATH_BETWEEN Return path between two nodes

    n_veh = length(search_tree.Node{1, 1}.trims);
        
    for i = 1:n_veh
                         
            cur_trim = cur_node.trims(i);
            next_trim = next_node.trims(i);
            
            maneuver = motion_graph.motionGraphList(i).maneuvers{cur_trim, next_trim};
            
            assert(~isempty(maneuver),'manuevers{%d, %d} is empty.',cur_trim, next_trim);
            
            x = cur_node.xs(i);
            y = cur_node.ys(i);
            yaw = cur_node.yaws(i);
            
            xs = maneuver.xs;
            ys = maneuver.ys;
            yaws = maneuver.yaws + yaw;       
            
            length_maneuver = length(xs);
            trims = cur_trim * ones(length_maneuver, 1);
            
            [xs, ys] = translate_global(yaw, x, y, xs, ys);
            
            xs = xs.';
            ys = ys.';

            search_paths(:,:,i) = [xs, ys, yaws, trims];

    end

end

