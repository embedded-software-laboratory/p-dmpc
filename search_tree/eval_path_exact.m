% MIT License
% 
% Copyright (c) 2021 Lehrstuhl Informatik 11 - RWTH Aachen University
% 
% Permission is hereby granted, free of charge, to any person obtaining a copy
% of this software and associated documentation files (the "Software"), to deal
% in the Software without restriction, including without limitation the rights
% to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
% copies of the Software, and to permit persons to whom the Software is
% furnished to do so, subject to the following conditions:
% 
% The above copyright notice and this permission notice shall be included in all
% copies or substantial portions of the Software.
% 
% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
% IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
% FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
% AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
% LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
% OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
% SOFTWARE.
% 
% This file is part of receding-horizon-graph-search.
% 
% Author: i11 - Embedded Software, RWTH Aachen University

function [iChop, evaluated_nodes, is_valid] = eval_path_exact(scenario, tree, root_to_node)
    is_valid = false;
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
        if ~tree.node{root_to_node(iNode)}(1,NodeInfo.exactEval)
            node_parent = tree.node{root_to_node(iNode-1)};
            for iVeh = 1 : scenario.nVeh
                t1 = node_parent(iVeh,NodeInfo.trim);
                t2 = tree.node{root_to_node(iNode)}(iVeh,NodeInfo.trim);
                maneuver = scenario.mpa.maneuvers{t1,t2};
                c = cos(node_parent(iVeh,NodeInfo.yaw));
                s = sin(node_parent(iVeh,NodeInfo.yaw));
                
                shape_x = c*maneuver.area(1,:) - s*maneuver.area(2,:) + node_parent(iVeh,NodeInfo.x);
                shape_y = s*maneuver.area(1,:) + c*maneuver.area(2,:) + node_parent(iVeh,NodeInfo.y);
                shapes{iVeh} = [shape_x;shape_y];
                
                % displacements(iVeh) = sqrt(maneuver.dx^2+maneuver.dy^2);
                % (1,NodeInfo) for x; (2,NodeInfo) for y
                % midpoints(:,iVeh) = [   node_parent(iVeh,NodeInfo.x)+(maneuver.dx*cos(node_parent(iVeh,NodeInfo.yaw))-maneuver.dy*sin(node_parent(iVeh,NodeInfo.yaw)))/2,...
                %                         node_parent(iVeh,NodeInfo.y)+(maneuver.dx*sin(node_parent(iVeh,NodeInfo.yaw))+maneuver.dy*cos(node_parent(iVeh,NodeInfo.yaw)))/2];
                
                % If collision, chop node and subtree
                if collision_with(iVeh, shapes, scenario)
                    % Chop parent if last sibling
                    iChop = iNode;
                    while ((numel(get_siblings(tree,root_to_node(iChop))) == 1) ...
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