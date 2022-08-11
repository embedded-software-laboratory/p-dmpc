function [is_valid, shapes] = eval_edge_exact(scenario, iter, tree, iNode, vehicle_obstacles, lanelet_boundary, lanelet_crossing_areas, method)
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

    if nargin < 8
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
        if strcmp(scenario.priority_option,'mixed_traffic_priority')
            % first check if mixed_traffic_priority is used to make a short circuit
            if ((scenario.vehicles(iVeh).ID == scenario.manual_vehicle_id) && scenario.manual_mpa_initialized && ~isempty(scenario.vehicles(iVeh).vehicle_mpa)) ...
                || ((scenario.vehicles(iVeh).ID == scenario.second_manual_vehicle_id) && scenario.second_manual_mpa_initialized && ~isempty(scenario.vehicles(iVeh).vehicle_mpa))
                mpa = scenario.vehicles(iVeh).vehicle_mpa;
                maneuver = mpa.maneuvers{t1,t2};
            else
                maneuver = scenario.mpa.maneuvers{t1,t2};
            end
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


        iStep = cK;

        if scenario.k>=67 && scenario.vehicles.ID==16
%             disp('')
            if t1==5 && t2==1
                disp('')
            end
        end

        switch method
            case 'sat'
                % check if collides with other vehicles' predicted trajectory or lanelets 
                if tree.k(iNode) == scenario.Hp
                    % with larger offset
                    shape_x_for_boundary_check = c*maneuver.area_large_offset(1,:) - s*maneuver.area_large_offset(2,:) + pX(iVeh);
                    shape_y_for_boundary_check = s*maneuver.area_large_offset(1,:) + c*maneuver.area_large_offset(2,:) + pY(iVeh);
                    shapes_for_boundary_check{iVeh} = [shape_x_for_boundary_check;shape_y_for_boundary_check];    
                else
                    % without offset
                    shapes_for_boundary_check{iVeh} = shapes_without_offset{iVeh};
                end
                if collision_with(iVeh, shapes, shapes_for_boundary_check, scenario, iStep)
                    is_valid = false;
                    return;
                end

            case 'InterX'
                assert(scenario.nVeh==1) % if not 1, code adaption is needed                
                % Note1: Shape must be closed!
                % Note2: The collision check order is important.
                % Normally, check collision with lanelet boundary last would be better.
                if InterX(shapes{iVeh}, vehicle_obstacles{iStep})
                    % check collision with vehicle obstacles
                    is_valid = false;
                    return
                end
                if InterX(shapes_without_offset{iVeh}, lanelet_crossing_areas)
                    % check collision with crossing area of lanelets
                    is_valid = false;
                    return
                end
                if InterX(shapes_without_offset{iVeh}, lanelet_boundary)
                    % check collision with lanelet obstacles
                    is_valid = false;
                    return
                else
                    if iStep==scenario.Hp
                        % at the last time step, check if vehicle could still move forward while not
                        % colliding with its lanelet boundary. This is done by
                        % checking the emergency left/right maneuvers. At least one of them should be
                        % collision-free with lanelet boundary.
                        x_Hp = c*maneuver.dx - s*maneuver.dy + pX(iVeh);
                        y_Hp = s*maneuver.dx + c*maneuver.dy + pY(iVeh);
                        yaw_Hp = maneuver.dyaw + pYaw;
                        s_Hp = sin(yaw_Hp);
                        c_Hp = cos(yaw_Hp);

                        area_straight = scenario.mpa.emergency_maneuvers{t2}.go_straight_area;
                        go_straight_x = c_Hp*area_straight(1,:) - s_Hp*area_straight(2,:) + x_Hp;
                        go_straight_y = s_Hp*area_straight(1,:) + c_Hp*area_straight(2,:) + y_Hp;
                        shape_go_straight = [go_straight_x;go_straight_y];
                        if InterX(shape_go_straight, lanelet_boundary)
                            % go straight is not collision-free with lanelet boundary
                            area_left = scenario.mpa.emergency_maneuvers{t2}.left_area_without_offset;
                            turn_left_x = c_Hp*area_left(1,:) - s_Hp*area_left(2,:) + x_Hp;
                            turn_left_y = s_Hp*area_left(1,:) + c_Hp*area_left(2,:) + y_Hp;
                            shape_turn_left = [turn_left_x;turn_left_y];
                            if InterX(shape_turn_left, lanelet_boundary)
                                % turn left most maneuver is also not collision-free with lanelet boundary
                                area_right = scenario.mpa.emergency_maneuvers{t2}.right_area_without_offset;
                                turn_right_x = c_Hp*area_right(1,:) - s_Hp*area_right(2,:) + x_Hp;
                                turn_right_y = s_Hp*area_right(1,:) + c_Hp*area_right(2,:) + y_Hp;
                                shape_turn_right = [turn_right_x;turn_right_y];
                                if InterX(shape_turn_right, lanelet_boundary)
                                    % turn right most maneuver is still not collision-free with lanelet boundary
                                    is_valid = false;
                                    return;
                                end
                            end
                        end
                    end
                end

            otherwise
                error("Please specify one of the following mthodes to check collision: 'sat', 'InterX'.")
        end
    end
end
