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

% set values to a node stored in a smaller node
function node = set_node(node,vehicle_filter,info)
    sub_node = info.tree.node{info.tree_path(2)};
    node(vehicle_filter,NodeInfo.k) = sub_node(:,NodeInfo.k);
    node(vehicle_filter,NodeInfo.trim) = sub_node(:,NodeInfo.trim);
    node(vehicle_filter,NodeInfo.x) = sub_node(:,NodeInfo.x);
    node(vehicle_filter,NodeInfo.y) = sub_node(:,NodeInfo.y);
    node(vehicle_filter,NodeInfo.yaw) = sub_node(:,NodeInfo.yaw);
    node(vehicle_filter,NodeInfo.g) = sub_node(:,NodeInfo.g);
    node(vehicle_filter,NodeInfo.h) = sub_node(:,NodeInfo.h);
end

