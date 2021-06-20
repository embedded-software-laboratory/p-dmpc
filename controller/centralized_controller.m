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

function [u, y_pred, info] = centralized_controller(scenario, iter)
    info = struct;
    info.vehicle_fullres_path = cell(scenario.nVeh,1);
    info.trim_indices = (-1)*ones(scenario.nVeh,1);
    info.subcontroller_runtime = zeros(scenario.nVeh,1);
    info.shapes = cell(scenario.nVeh,scenario.Hp);
    info.next_node = node(-1, zeros(scenario.nVeh,1), zeros(scenario.nVeh,1), zeros(scenario.nVeh,1), zeros(scenario.nVeh,1), -1, -1);
    info.n_expanded = 0;
    
    sub_controller = @(scenario, iter)...
        graph_search(scenario, iter);
    % falsifies controller_runtime slightly
    subcontroller_timer = tic;
    [u, y_pred, info_v] = sub_controller(scenario, iter);
    
    % prepare output data
    info.tree = info_v.tree; % only for node explorationslee
    info.subcontroller_runtime = toc(subcontroller_timer);
    info.n_expanded = info.n_expanded + numel(info_v.tree.node);
    info.next_node = set_node(info.next_node,1:scenario.nVeh,info_v);
    info.shapes = info_v.shapes;
    info.vehicle_fullres_path = path_between(info_v.tree.node{info_v.tree_path(1)},info_v.tree.node{info_v.tree_path(2)},info_v.tree,scenario.mpa);
    info.trim_indices = info_v.trim_indices;
end

