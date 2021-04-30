function [iChop, evaluated_nodes, search_finished] = eval_path_exact(scenario, theTree, root_to_node)
    search_finished = false;
    iChop = -1;
    evaluated_nodes = false(size(root_to_node));
    % Collision check on path from root
    nNodes = numel(root_to_node);
    shapes = cell(scenario.nVeh,1);
    % maneuver shapes correspond to movement TO node, so start from 2
    for iNode = 2:nNodes
        % Check if exact evaluation needs to be done
        % displacements = zeros(1,scenario.nVeh);
        % midpoints = zeros(2,scenario.nVeh);
        if ~theTree.Node{root_to_node(iNode)}(1,theTree.idx.exactEval)
            node_parent = theTree.Node{root_to_node(iNode-1)};
            for iVeh = 1 : scenario.nVeh
                t1 = node_parent(iVeh,theTree.idx.trim);
                t2 = theTree.Node{root_to_node(iNode)}(iVeh,theTree.idx.trim);
                maneuver = scenario.mpa.maneuvers{t1,t2};
                c = cos(node_parent(iVeh,theTree.idx.yaw));
                s = sin(node_parent(iVeh,theTree.idx.yaw));
                
                shape_x = c*maneuver.area(1,:) - s*maneuver.area(2,:) + node_parent(iVeh,theTree.idx.x);
                shape_y = s*maneuver.area(1,:) + c*maneuver.area(2,:) + node_parent(iVeh,theTree.idx.y);
                shapes{iVeh} = [shape_x;shape_y];
                
                % displacements(iVeh) = sqrt(maneuver.dx^2+maneuver.dy^2);
                % (1,theTree.idx) for x; (2,theTree.idx) for y
                % midpoints(:,iVeh) = [   node_parent(iVeh,theTree.idx.x)+(maneuver.dx*cos(node_parent(iVeh,theTree.idx.yaw))-maneuver.dy*sin(node_parent(iVeh,theTree.idx.yaw)))/2,...
                %                         node_parent(iVeh,theTree.idx.y)+(maneuver.dx*sin(node_parent(iVeh,theTree.idx.yaw))+maneuver.dy*cos(node_parent(iVeh,theTree.idx.yaw)))/2];
                
                % If collision, chop node and subtree
                if collision_with(iVeh, shapes, scenario)
                    % Chop parent if last sibling
                    iChop = iNode;
                    while ((numel(getsiblings(theTree,root_to_node(iChop))) == 1) ...
                            && iChop ~= 1)
                        iChop = iChop - 1;
                    end
                    return
                end
            end
            evaluated_nodes(iNode) = true;
        end
    end
    search_finished = true;
end