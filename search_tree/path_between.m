function search_path = path_between(cur_node,next_node, tree, mpa)
%PATH_BETWEEN Return path as a cell array between two nodes
    n_veh = length(tree.node{1, 1}(:,NodeInfo.trim));
    search_path = cell(1, n_veh);
    for iVeh = 1:n_veh        
        maneuver = mpa.maneuvers{cur_node(iVeh,NodeInfo.trim), next_node(iVeh,NodeInfo.trim)};
        assert(~isempty(maneuver),'manuevers{%d, %d} is empty.',cur_node(iVeh,NodeInfo.trim), next_node(iVeh,NodeInfo.trim));
        
        x = cur_node(iVeh,NodeInfo.x);
        y = cur_node(iVeh,NodeInfo.y);
        yaw = cur_node(iVeh,NodeInfo.yaw);
        
        xs = maneuver.xs;
        ys = maneuver.ys;
        yaws = maneuver.yaws + yaw;       
        
        length_maneuver = length(xs);
        trims = cur_node(iVeh,NodeInfo.trim) * ones(length_maneuver, 1);
        
        [xs, ys] = translate_global(yaw, x, y, xs, ys);
        
        search_path(iVeh) = {[xs', ys', yaws', trims]};
    end
end

