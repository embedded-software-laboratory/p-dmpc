function [is_valid, shapes, is_collide_with_reachableSets] = eval_edge_exact(scenario, tree, iNode)
% EVAL_EDGE_EXACT   Evaluate if step is valid.

    is_valid = true;
    % maneuver shapes correspond to movement TO node
    node_id_parent = get_parent(tree, iNode);
    shapes = cell(scenario.nVeh,1);
    is_collide_with_reachableSets = false;
    if ~node_id_parent % root node without parent
        return;
    end

    pX     = tree.x(:,node_id_parent);
    pY     = tree.y(:,node_id_parent);
    pYaw   = tree.yaw(:,node_id_parent);
    pTrim  = tree.trim(:,node_id_parent);

    cTrim  = tree.trim(:,iNode);
    cK     = tree.k(:,iNode);

    for iVeh = 1 : scenario.nVeh
        t1 = pTrim(iVeh);
        t2 = cTrim(iVeh);
        maneuver = scenario.mpa.maneuvers{t1,t2};
        c = cos(pYaw(iVeh));
        s = sin(pYaw(iVeh));
        
        shape_x = c*maneuver.area(1,:) - s*maneuver.area(2,:) + pX(iVeh);
        shape_y = s*maneuver.area(1,:) + c*maneuver.area(2,:) + pY(iVeh);
        shapes{iVeh} = [shape_x;shape_y];
        
        iStep = cK;

        % check if collides with other vehicles' predicted trajectory or
        % lanelets 
        collision  = collision_with(iVeh, shapes, scenario, iStep);
        if collision
            is_valid = false;
            return;
        end

        % check if collides with other vehicles' reachable sets
        if iStep<=scenario.Hp  % only check the reachable sets of maximal two time step to avoid being too conservativeness
            collision_reachableSets  = collision_with_reachableSets(iVeh, shapes, scenario, iStep);
            if collision_reachableSets
                is_valid = false;
                is_collide_with_reachableSets = true;
                return;
            end
        end

    end
end
