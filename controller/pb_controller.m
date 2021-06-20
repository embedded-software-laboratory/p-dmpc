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

function [u, y_pred, info] = pb_controller(scenario, iter)
    assert( ~isempty(scenario.adjacency) )

    % determine planning levels
    if scenario.assignPrios || isempty(scenario.directed_coupling)
        [isDAG, topo_groups] = topological_sorting_coloring(scenario.adjacency);
    else
        [isDAG, topo_groups] = kahn(scenario.directed_coupling);
    end
    assert( isDAG, 'Coupling matrix is not a DAG' );
    
    % get planning groups and their predecessors
    groups = PB_predecessor_groups(topo_groups);

    y_pred = cell(scenario.nVeh,1);
    u = zeros(scenario.nVeh,1);
    info = struct;
    info.vehicle_fullres_path = cell(scenario.nVeh,1);
    info.trim_indices = (-1)*ones(scenario.nVeh,1);
    info.subcontroller_runtime = zeros(scenario.nVeh,1);
    info.shapes = cell(scenario.nVeh,scenario.Hp);
    info.next_node = node(-1, zeros(scenario.nVeh,1), zeros(scenario.nVeh,1), zeros(scenario.nVeh,1), zeros(scenario.nVeh,1), -1, -1);
    info.n_expanded = 0;
    
    sub_controller = @(scenario, iter)...
        graph_search(scenario, iter);
    
    for grp_idx = 1:length(groups)
        group = groups(grp_idx);
        for grp_member_idx = 1:length(group.members) 
            subcontroller_timer = tic;
            vehicle_idx = group.members(grp_member_idx);

            
            % Filter out vehicles with lower or same priority.
            priority_filter = false(1,scenario.nVeh);
            priority_filter(group.predecessors) = true; % keep all with higher priority
            priority_filter(vehicle_idx) = true; % keep self
            scenario_filtered = filter_scenario(scenario, priority_filter);
            iter_filtered = filter_iter(iter, priority_filter);

            self_index = sum(priority_filter(1:vehicle_idx));        
            v2o_filter = true(1,scenario_filtered.nVeh);
            v2o_filter(self_index) = false;

            % add predicted trajecotries as obstacle
            [scenario_v, iter_v] = vehicles_as_obstacles(scenario_filtered, iter_filtered, v2o_filter, info.shapes(group.predecessors,:));
    
            % execute sub controller for 1-veh scenario
            [u_v,y_pred_v,info_v] = sub_controller(scenario_v, iter_v);
            
            % prepare output data
            info.subcontroller_runtime(vehicle_idx) = toc(subcontroller_timer);
            info.n_expanded = info.n_expanded + numel(info_v.tree.node);
            info.next_node = set_node(info.next_node,[vehicle_idx],info_v);
            info.shapes(vehicle_idx,:) = info_v.shapes(:);
            info.vehicle_fullres_path(vehicle_idx) = path_between(info_v.tree.node{info_v.tree_path(1)},info_v.tree.node{info_v.tree_path(2)},info_v.tree,scenario.mpa);
            info.trim_indices(vehicle_idx) = info_v.trim_indices(1);
            y_pred{vehicle_idx,1} = y_pred_v{:};
            u(vehicle_idx) = u_v(1);
        end
    end
    
end
