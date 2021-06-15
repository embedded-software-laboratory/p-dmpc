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

function [is_valid, shapes] = eval_edge_exact(scenario, tree, node_id)
    is_valid = true;
    % maneuver shapes correspond to movement TO node
    node_id_parent = get_parent(tree, node_id);
    shapes = cell(scenario.nVeh,1);
    if ~node_id_parent % root node without parent
        return;
    end
    node_parent = tree.node{node_id_parent};
    node_child = tree.node{node_id};
    for iVeh = 1 : scenario.nVeh
        t1 = node_parent(iVeh,NodeInfo.trim);
        t2 = node_child(iVeh,NodeInfo.trim);
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
        
        iStep = node_child(1,NodeInfo.k);
        
        % If collision, chop node and subtree
        if collision_with(iVeh, shapes, scenario, iStep)
            is_valid = false;
            return;
        end
    end
end