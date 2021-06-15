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

function search_path = path_between(cur_node,next_node, tree, mpa)
%PATH_BETWEEN Return path as a cell array between two nodes
    n_veh = length(tree.node{1, 1}(:,NodeInfo.trim));
    search_path = cell(1, n_veh);
    for iVeh = 1:n_veh        
        maneuver = mpa.maneuvers{cur_node(iVeh,NodeInfo.trim), next_node(iVeh,NodeInfo.trim)};
        assert(~isempty(maneuver),'manuevers{%d, %d} is empty.',cur_node(iVeh,NodeInfo.trim), next_node(iVeh,NodeInfo.trim));
        
        x = cur_node(iVeh,NodeInfo.x);
        y = cur_node(iVeh,NodeInfo.y);
        yaw = cur_node(iVeh,NodeInfo.yaw);
        
        xs = maneuver.xs;
        ys = maneuver.ys;
        yaws = maneuver.yaws + yaw;       
        
        length_maneuver = length(xs);
        trims = cur_node(iVeh,NodeInfo.trim) * ones(length_maneuver, 1);
        
        [xs, ys] = translate_global(yaw, x, y, xs, ys);
        
        search_path(iVeh) = {[xs', ys', yaws', trims]};
    end
end

