function [is_valid, shapes] = eval_edge_exact(scenario, tree, iNode)
% EVAL_EDGE_EXACT   Evaluate if step is valid.

    is_valid = true;
    % maneuver shapes correspond to movement TO node
    node_id_parent = get_parent(tree, iNode);
    shapes = cell(scenario.nVeh,1);
    shapes_without_offset = cell(scenario.nVeh,1);
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

        % if current vehicle is manual vehicle and its MPA is already initialized, choose the corresponding MPA
        if ((scenario.vehicles(iVeh).vehicle_id == scenario.manual_vehicle_id) && scenario.manual_mpa_initialized && ~isempty(scenario.vehicles(iVeh).vehicle_mpa)) ...
            || ((scenario.vehicles(iVeh).vehicle_id == scenario.second_manual_vehicle_id) && scenario.second_manual_mpa_initialized && ~isempty(scenario.vehicles(iVeh).vehicle_mpa))
            mpa = scenario.vehicles(iVeh).vehicle_mpa;
            maneuver = mpa.maneuvers{t1,t2};
        else
            maneuver = scenario.mpa.maneuvers{t1,t2};
        end
    
        c = cos(pYaw(iVeh));
        s = sin(pYaw(iVeh));
        
        shape_x = c*maneuver.area(1,:) - s*maneuver.area(2,:) + pX(iVeh);
        shape_y = s*maneuver.area(1,:) + c*maneuver.area(2,:) + pY(iVeh);
        shapes{iVeh} = [shape_x;shape_y];
        
        if tree.k(iNode) == scenario.Hp
            shape_x_without_offset = c*maneuver.area_boundary_check(1,:) - s*maneuver.area_boundary_check(2,:) + pX(iVeh);
            shape_y_without_offset = s*maneuver.area_boundary_check(1,:) + c*maneuver.area_boundary_check(2,:) + pY(iVeh);
            shapes_without_offset{iVeh} = [shape_x_without_offset;shape_y_without_offset];    
        else
            shape_x_without_offset = c*maneuver.area_without_offset(1,:) - s*maneuver.area_without_offset(2,:) + pX(iVeh);
            shape_y_without_offset = s*maneuver.area_without_offset(1,:) + c*maneuver.area_without_offset(2,:) + pY(iVeh);
            shapes_without_offset{iVeh} = [shape_x_without_offset;shape_y_without_offset];
        
        end
        
        
        iStep = cK;

        if collision_with(iVeh, shapes, shapes_without_offset, scenario, iStep)
            is_valid = false;
            return;
        end
    end
end
