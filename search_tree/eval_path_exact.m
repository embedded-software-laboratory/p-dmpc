function [theTree, open_nodes, open_values, search_finished] = eval_path_exact(scenario, theTree, open_nodes, open_values, cur_node_id)
    % Collision check on path from root
    p = fliplr(pathtoroot(theTree, cur_node_id));
    nNodes = numel(p);
    % maneuver shapes correspond to movement TO node, so start from 2
    for iNode = 2:nNodes
        % Check if exact evaluation needs to be done
        displacements = zeros(1,scenario.nVeh);
        midpoints = zeros(2,scenario.nVeh);
        if ~theTree.Node{p(iNode)}.exact_eval
            node_parent = theTree.Node{p(iNode-1)};
            for iVeh = 1 : scenario.nVeh
                t1 = node_parent.trims(iVeh);
                t2 = theTree.Node{p(iNode)}.trims(iVeh);
                maneuver = scenario.combined_graph.motionGraphList(iVeh).maneuvers{t1,t2};
                [shape_x, shape_y] = translate_global(...
                    node_parent.yaws(iVeh),...
                    node_parent.xs(iVeh),...
                    node_parent.ys(iVeh),...
                    maneuver.area(1,:),...
                    maneuver.area(2,:)...
                );
                theTree.Node{p(iNode)}.shapes{iVeh} = [shape_x;shape_y];
                displacements(iVeh) = sqrt(maneuver.dx^2+maneuver.dy^2);
                % (1,idx) for x; (2,idx) for y
                midpoints(:,iVeh) = [   node_parent.xs(iVeh)+(maneuver.dx*cos(node_parent.yaws(iVeh))-maneuver.dy*sin(node_parent.yaws(iVeh)))/2,...
                                        node_parent.ys(iVeh)+(maneuver.dx*sin(node_parent.yaws(iVeh))+maneuver.dy*cos(node_parent.yaws(iVeh)))/2];
                
                % If collision, chop node and subtree
                if collision_with(iVeh, theTree.Node{p(iNode)}.shapes, displacements, midpoints, scenario)
                    % Chop parent if last sibling
                    iChop = iNode;
                    while ((numel(getsiblings(theTree,p(iChop))) == 1) ...
                            && iChop ~= 1)
                        iChop = iChop - 1;
                    end
                    [theTree, choppedNodes] = chop(theTree,p(iChop));
                    list_indices_to_del = [];
                    for cNode = choppedNodes
                        list_indices_to_del = [list_indices_to_del, find(open_nodes==cNode,1)];
                    end
                    choppedNodes = sort(choppedNodes,'descend');
                    for idx = choppedNodes
                        open_nodes(open_nodes>idx) = open_nodes(open_nodes>idx)-1;
                    end
                    % open_nodes = findleaves(theTree);
                    open_nodes(list_indices_to_del) = [];
                    open_values(list_indices_to_del) = [];
                    search_finished = false;
                    return
                end
            end
            theTree.Node{p(iNode)}.exact_eval = true;
        end
    end
    search_finished = true;
end