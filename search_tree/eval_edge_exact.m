function [is_valid, shapes] = eval_edge_exact(scenario, tree, iNode, vehicle_obstacles, lanelet_boundary, lanelet_crossing_areas, method)
% EVAL_EDGE_EXACT   Evaluate if step is valid.
% 
% INPUT:
%   scenario: instance of the class Scenario
% 
%   tree: instance of the class Tree
% 
%   iNode: tree node to be checked
% 
%   method: string, method to check collision, currently two methods are available:
%       1. 'sat': separating axis theorem
%       2. 'InterX': an algorithm to fast check intersection of curves
% 
% OUTPUT:
%   is_valid: logical, indicates whether the selected node is collision-free
% 
%   shapes: occupied area of the vehicle if the selected node is chosen
% 

    if nargin < 7
        method = 'sat'; % use the default method, separating axis theorem, to check if collide
    end

    is_valid = true;
    % maneuver shapes correspond to movement TO node
    node_id_parent = get_parent(tree, iNode);
    shapes = cell(scenario.nVeh,1);
    shapes_without_offset = cell(scenario.nVeh,1);
    shapes_for_boundary_check = cell(scenario.nVeh,1);
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
        if ((scenario.vehicles(iVeh).ID == scenario.manual_vehicle_id) && scenario.manual_mpa_initialized && ~isempty(scenario.vehicles(iVeh).vehicle_mpa)) ...
            || ((scenario.vehicles(iVeh).ID == scenario.second_manual_vehicle_id) && scenario.second_manual_mpa_initialized && ~isempty(scenario.vehicles(iVeh).vehicle_mpa))
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

        shape_x_without_offset = c*maneuver.area_without_offset(1,:) - s*maneuver.area_without_offset(2,:) + pX(iVeh);
        shape_y_without_offset = s*maneuver.area_without_offset(1,:) + c*maneuver.area_without_offset(2,:) + pY(iVeh);
        shapes_without_offset{iVeh} = [shape_x_without_offset;shape_y_without_offset];    

        % check if collides with other vehicles' predicted trajectory or lanelets 
        if tree.k(iNode) == scenario.Hp
            % with larger offset
            shape_x_for_boundary_check = c*maneuver.area_boundary_check(1,:) - s*maneuver.area_boundary_check(2,:) + pX(iVeh);
            shape_y_for_boundary_check = s*maneuver.area_boundary_check(1,:) + c*maneuver.area_boundary_check(2,:) + pY(iVeh);
            shapes_for_boundary_check{iVeh} = [shape_x_for_boundary_check;shape_y_for_boundary_check];    
        else
            % without offset
            shapes_for_boundary_check{iVeh} = shapes_without_offset{iVeh};
        end

        iStep = cK;

%         if scenario.k>4 && scenario.vehicles.ID==18 && iStep>=5
%             disp('')
%         end

        switch method
            case 'sat'
                if collision_with(iVeh, shapes, shapes_for_boundary_check, scenario, iStep)
                    is_valid = false;
                    return;
                end

            case 'InterX'
                assert(scenario.nVeh==1) % if not 1, code adaption is needed
                % repeat the last column to enclose the shape
                if InterX([shapes{iVeh},shapes{iVeh}(:,1)], vehicle_obstacles{iStep})
                    % check collision with vehicle obstacles
                    is_valid = false;
                    return
                end
                if InterX([shapes_without_offset{iVeh},shapes_without_offset{iVeh}(:,1)], lanelet_crossing_areas{iStep})
                    % check collision with crossing area of lanelets
                    is_valid = false;
                    return
                end
                if InterX([shapes_for_boundary_check{iVeh},shapes_for_boundary_check{iVeh}(:,1)], lanelet_boundary)
                    % check collision with lanelet obstacles
                    is_valid = false;
                    return
                end

            otherwise
                error("Please specify one of the following mthodes to check collision: 'sat', 'InterX'.")
        end
    end
end
