function search_path = path_between(cur_node,next_node, search_tree, mpa)
%PATH_BETWEEN Return path as a cell array between two nodes
    n_veh = length(search_tree.Node{1, 1}(:,search_tree.idx.trim));
    search_path = cell(1, n_veh);
    for iVeh = 1:n_veh        
        if(cur_node(iVeh,search_tree.idx.x) ~= next_node(iVeh,search_tree.idx.x) || cur_node(iVeh,search_tree.idx.y) ~= next_node(iVeh,search_tree.idx.y))
            maneuver = mpa.maneuvers{cur_node(iVeh,search_tree.idx.trim), next_node(iVeh,search_tree.idx.trim)};
            assert(~isempty(maneuver),'manuevers{%d, %d} is empty.',cur_node(iVeh,search_tree.idx.trim), next_node(iVeh,search_tree.idx.trim));
            
            x = cur_node(iVeh,search_tree.idx.x);
            y = cur_node(iVeh,search_tree.idx.y);
            yaw = cur_node(iVeh,search_tree.idx.yaw);
            
            xs = maneuver.xs;
            ys = maneuver.ys;
            yaws = maneuver.yaws + yaw;       
            
            length_maneuver = length(xs);
            trims = cur_node(iVeh,search_tree.idx.trim) * ones(length_maneuver, 1);
            
            [xs, ys] = translate_global(yaw, x, y, xs, ys);
            
            search_path(iVeh) = {[xs', ys', yaws', trims]};
        end
    end
end
