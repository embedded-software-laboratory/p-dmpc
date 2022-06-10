function [is_valid, shapes] = eval_edge_exact(scenario, tree, iNode, vehicle_obstacles, lanelet_obstacles, method)
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

    if nargin < 6
        method = 'sat'; % use the default method, separating axis theorem, to check if collide
    end

    is_valid = true;
    % maneuver shapes correspond to movement TO node
    node_id_parent = get_parent(tree, iNode);
    shapes = cell(scenario.nVeh,1);
    shapes_for_lanelet_check = cell(scenario.nVeh,1);
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
        

        % check if collides with other vehicles' predicted trajectory or lanelets 
        if tree.k(iNode) == scenario.Hp
            % with larger offset
            shape_x_for_lanelet_check = c*maneuver.area_boundary_check(1,:) - s*maneuver.area_boundary_check(2,:) + pX(iVeh);
            shape_y_for_lanelet_check = s*maneuver.area_boundary_check(1,:) + c*maneuver.area_boundary_check(2,:) + pY(iVeh);
            shapes_for_lanelet_check{iVeh} = [shape_x_for_lanelet_check;shape_y_for_lanelet_check];    
        else
            % without offset
            shape_x_for_lanelet_check = c*maneuver.area_without_offset(1,:) - s*maneuver.area_without_offset(2,:) + pX(iVeh);
            shape_y_for_lanelet_check = s*maneuver.area_without_offset(1,:) + c*maneuver.area_without_offset(2,:) + pY(iVeh);
            shapes_for_lanelet_check{iVeh} = [shape_x_for_lanelet_check;shape_y_for_lanelet_check];
        end

        iStep = cK;

%         if scenario.k>4 && scenario.vehicles.ID==18 && iStep>=5
%             disp('')
%         end

        switch method
            case 'sat'
                if collision_with(iVeh, shapes, shapes_for_lanelet_check, scenario, iStep)
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
                if InterX([shapes_for_lanelet_check{iVeh},shapes_for_lanelet_check{iVeh}(:,1)], lanelet_obstacles{iStep})
                    % check collision with lanelet obstacles
                    is_valid = false;
                    return
                end

            otherwise
                error("Please specify one of the following mthodes to check collision: 'sat', 'InterX'.")
        end
    end
end
