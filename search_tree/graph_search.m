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

function [u, y_pred, info] = graph_search(scenario, iter)
    info = struct;
    shapes_tmp = cell(scenario.nVeh,0);
    % Create tree with root node
    init_depth = 0;
    g_values = zeros(scenario.nVeh,1);
    h_values = zeros(scenario.nVeh,1);
    cur_node = node(...
        init_depth, ...
        iter.trim_indices, ...
        iter.x0(:,1), ...
        iter.x0(:,2), ...
        iter.x0(:,3), ...
        g_values, ...
        h_values...
    );
    info.tree = Tree(cur_node);
    
    % Array storing ids of nodes that may be expanded
    open_nodes = 1;
    open_values = sum_values(info.tree, open_nodes);
        
    % Expand leaves of tree until depth or target is reached or until there 
    % are no leaves
    while true
        % Choose cheapest node for expansion
        if numel(open_nodes) == 0
            ME = MException( ...
                'MATLAB:graph_search:tree_exhausted' ...
                ,'No more open nodes to explore' ...
            );
            throw(ME);
        end
        
        cur_node_id = open_nodes(1);
        cur_node = info.tree.node{cur_node_id};
        
        % remove parent node
        open_nodes(1) = [];
        open_values(1) = [];

        % Eval edge 
        [is_valid, shapes] = eval_edge_exact(scenario, info.tree, cur_node_id);
        if ~is_valid
            % could remove node from tree here
            continue
        end
        shapes_tmp(:,cur_node_id) = shapes;
        if cur_node(1,NodeInfo.k) == scenario.Hp
            y_pred = return_path_to(cur_node_id, info.tree, scenario.mpa);
            u = 0;
            info.shapes = return_path_area(shapes_tmp, info.tree, cur_node_id);
            info.tree_path = fliplr(path_to_root(info.tree, cur_node_id));
            info.trim_indices = info.tree.node{info.tree_path(2)}(:,NodeInfo.trim);
            info.open_nodes = open_nodes;
            info.open_values = open_values;
            break
        else
            % Expand chosen node
            expanded_nodes = expand_node(...
                scenario...
                ,iter...
                ,cur_node...
            );
            parents = cur_node_id*ones(numel(expanded_nodes),1);
            [info.tree, new_open_nodes] = info.tree.add_nodes(parents, expanded_nodes);
            % add child nodes
            open_nodes = [open_nodes, new_open_nodes]; %#ok<AGROW>
            open_values = [open_values, sum_values(info.tree, new_open_nodes)]; %#ok<AGROW>
            [open_nodes, open_values] = sort_open_list(open_nodes, open_values);

            % % plot exploration
            % info.plot = visualize_exploration(scenario, info.tree, info.plot);
        end
    end
end