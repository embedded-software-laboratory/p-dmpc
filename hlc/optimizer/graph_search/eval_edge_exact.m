function [is_valid, shapes] = eval_edge_exact(iter, options, mpa, tree, iNode, vehicle_obstacles, hdv_obstacles, lanelet_boundary, lanelet_crossing_areas, method)
    % EVAL_EDGE_EXACT   Evaluate if step is valid.
    %
    % INPUT:
    %   options: instance of the class Config
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
    shapes = cell(iter.amount, 1);
    shapes_without_offset = cell(iter.amount, 1);
    shapes_for_boundary_check = cell(iter.amount, 1);

    if ~node_id_parent % root node without parent
        return;
    end

    pX = tree.x(:, node_id_parent);
    pY = tree.y(:, node_id_parent);
    pYaw = tree.yaw(:, node_id_parent);
    pTrim = tree.trim(:, node_id_parent);

    cTrim = tree.trim(:, iNode);
    cK = tree.k(:, iNode);

    for iVeh = 1:iter.amount
        t1 = pTrim(iVeh);
        t2 = cTrim(iVeh);

        maneuver = mpa.maneuvers{t1, t2};

        c = cos(pYaw(iVeh));
        s = sin(pYaw(iVeh));

        shape_x = c * maneuver.area(1, :) - s * maneuver.area(2, :) + pX(iVeh);
        shape_y = s * maneuver.area(1, :) + c * maneuver.area(2, :) + pY(iVeh);
        shapes{iVeh} = [shape_x; shape_y];

        shape_x_without_offset = c * maneuver.area_without_offset(1, :) - s * maneuver.area_without_offset(2, :) + pX(iVeh);
        shape_y_without_offset = s * maneuver.area_without_offset(1, :) + c * maneuver.area_without_offset(2, :) + pY(iVeh);
        shapes_without_offset{iVeh} = [shape_x_without_offset; shape_y_without_offset];

        if tree.k(iNode) == options.Hp
            % with larger offset
            shape_x_for_boundary_check = c * maneuver.area_large_offset(1, :) - s * maneuver.area_large_offset(2, :) + pX(iVeh);
            shape_y_for_boundary_check = s * maneuver.area_large_offset(1, :) + c * maneuver.area_large_offset(2, :) + pY(iVeh);
            shapes_for_boundary_check{iVeh} = [shape_x_for_boundary_check; shape_y_for_boundary_check];
        else
            % without offset
            shapes_for_boundary_check{iVeh} = shapes_without_offset{iVeh};
        end

        iStep = cK;

        switch method
            case 'sat'
                % check if collides with other vehicles' predicted trajectory or lanelets

                if collision_with(iter, iVeh, shapes, shapes_for_boundary_check, iStep)
                    is_valid = false;
                    return;
                end

            case 'InterX'
                assert(iter.amount == 1) % if not 1, code adaption is needed
                % Note1: Shape must be closed!
                % Note2: The collision check order is important.
                % Normally, check collision with lanelet boundary last would be better.
                if options.coupling ~= CouplerStrategies.no_coupling
                    % In free flow mode, vehicles do not need to consider
                    % other vehicles
                    if InterX(shapes{iVeh}, vehicle_obstacles{iStep})
                        % check collision with vehicle obstacles
                        is_valid = false;
                        return
                    end

                    if options.amount > 1 && InterX(shapes_without_offset{iVeh}, lanelet_crossing_areas)
                        % check collision with crossing area of lanelets
                        is_valid = false;
                        return
                    end

                    is_hdv_obstacle = ~all(all(isnan(hdv_obstacles{iStep})));

                    if (is_hdv_obstacle && ...
                            InterX(shapes{iVeh}, hdv_obstacles{iStep}) ...
                        )
                        % check collision with manual vehicle obstacles
                        is_valid = false;
                        return
                    end

                end

                % check collision with lanelet obstacles
                if InterX(shapes_for_boundary_check{iVeh}, lanelet_boundary)
                    is_valid = false;
                    return
                end

            otherwise
                error("Please specify one of the following mthodes to check collision: 'sat', 'InterX'.")
        end

    end

end
