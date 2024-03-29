function [iChop, evaluated_nodes, is_valid] = eval_path_exact(iter, scenario, tree, root_to_node)
    % EVAL_PATH_EXACT   Evaluate if a whole path is valid.

    is_valid = false;
    iChop = -1;
    evaluated_nodes = false(size(root_to_node));
    % Collision check on path from root
    nNodes = numel(root_to_node);
    shapes = cell(iter.amount, 1);
    % maneuver shapes correspond to movement TO node, so start from 2
    for iNode = 2:nNodes
        % Check if exact evaluation needs to be done
        % displacements = zeros(1,scenario.options.amount);
        % midpoints = zeros(2,scenario.options.amount);
        if ~tree.node{root_to_node(iNode)}(1, NodeInfo.exactEval)
            node_parent = tree.node{root_to_node(iNode - 1)};

            for iVeh = 1:iter.amount
                t1 = node_parent(iVeh, NodeInfo.trim);
                t2 = tree.node{root_to_node(iNode)}(iVeh, NodeInfo.trim);

                maneuver = scenario.mpa.maneuvers{t1, t2};

                c = cos(node_parent(iVeh, NodeInfo.yaw));
                s = sin(node_parent(iVeh, NodeInfo.yaw));

                shape_x = c * maneuver.area(1, :) - s * maneuver.area(2, :) + node_parent(iVeh, NodeInfo.x);
                shape_y = s * maneuver.area(1, :) + c * maneuver.area(2, :) + node_parent(iVeh, NodeInfo.y);
                shapes{iVeh} = [shape_x; shape_y];

                % displacements(iVeh) = sqrt(maneuver.dx^2+maneuver.dy^2);
                % (1,NodeInfo) for x; (2,NodeInfo) for y
                % midpoints(:,iVeh) = [   node_parent(iVeh,NodeInfo.x)+(maneuver.dx*cos(node_parent(iVeh,NodeInfo.yaw))-maneuver.dy*sin(node_parent(iVeh,NodeInfo.yaw)))/2,...
                %                         node_parent(iVeh,NodeInfo.y)+(maneuver.dx*sin(node_parent(iVeh,NodeInfo.yaw))+maneuver.dy*cos(node_parent(iVeh,NodeInfo.yaw)))/2];

                % If collision, chop node and subtree
                if collision_with(iVeh, shapes, scenario)
                    % Chop parent if last sibling
                    iChop = iNode;

                    while ((numel(get_siblings(tree, root_to_node(iChop))) == 1) ...
                            && iChop ~= 1)
                        iChop = iChop - 1;
                    end

                    return
                end

            end

            evaluated_nodes(iNode) = true;
        end

    end

    is_valid = true;
end
