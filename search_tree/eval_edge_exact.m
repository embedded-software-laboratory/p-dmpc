function is_valid = eval_edge_exact(scenario, search_tree, node_id)
    is_valid = true;
    % maneuver shapes correspond to movement TO node
    node_id_parent = getparent(search_tree, node_id);
    if ~node_id_parent % root node without parent
        return;
    end
    node_parent = search_tree.Node{node_id_parent};
    node_child = search_tree.Node{node_id};
    shapes = cell(scenario.nVeh,1);
    for iVeh = 1 : scenario.nVeh
        t1 = node_parent(iVeh,search_tree.idx.trim);
        t2 = node_child(iVeh,search_tree.idx.trim);
        maneuver = scenario.mpa.maneuvers{t1,t2};
        c = cos(node_parent(iVeh,search_tree.idx.yaw));
        s = sin(node_parent(iVeh,search_tree.idx.yaw));
        
        shape_x = c*maneuver.area(1,:) - s*maneuver.area(2,:) + node_parent(iVeh,search_tree.idx.x);
        shape_y = s*maneuver.area(1,:) + c*maneuver.area(2,:) + node_parent(iVeh,search_tree.idx.y);
        shapes{iVeh} = [shape_x;shape_y];
        
        % displacements(iVeh) = sqrt(maneuver.dx^2+maneuver.dy^2);
        % (1,search_tree.idx) for x; (2,search_tree.idx) for y
        % midpoints(:,iVeh) = [   node_parent(iVeh,search_tree.idx.x)+(maneuver.dx*cos(node_parent(iVeh,search_tree.idx.yaw))-maneuver.dy*sin(node_parent(iVeh,search_tree.idx.yaw)))/2,...
        %                         node_parent(iVeh,search_tree.idx.y)+(maneuver.dx*sin(node_parent(iVeh,search_tree.idx.yaw))+maneuver.dy*cos(node_parent(iVeh,search_tree.idx.yaw)))/2];
        
        % If collision, chop node and subtree
        if collision_with(iVeh, shapes, scenario)
            is_valid = false;
            return;
        end
    end
end