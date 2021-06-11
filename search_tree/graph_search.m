function [u, y_pred, info] = graph_search(scenario, iter)
    info = struct;

    % Create tree with root node
    % TODO Choose trim_indices depending on measurement
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
    % TODO Preallocate
    open_nodes = 1;
    open_values = sum_values(info.tree, open_nodes);
    
    % Array storing ids of nodes that were visited
    visited_nodes = [];
    
    % Expand leaves of tree until depth or target is reached or until there 
    % are no leaves
    while true
        % Choose cheapest node for expansion
        if numel(open_nodes) == 0
            ME = MException('graph_search:tree_exhausted' ...
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
        is_valid = eval_edge_exact(scenario, info.tree, cur_node_id);
        if ~is_valid
            % could remove node from tree here
            continue
        end
        if cur_node(1,NodeInfo.depth) == scenario.Hp
            y_pred = return_path_to(cur_node_id, info.tree, scenario.mpa);
            % TODO u
            u = 0;
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
            [info.tree, new_open_nodes] = info.tree.addnnodes(parents, expanded_nodes);
            % add child nodes
            open_nodes = [open_nodes, new_open_nodes];
            open_values = [open_values, sum_values(info.tree, new_open_nodes)];
            [open_nodes, open_values] = sort_open_list(open_nodes, open_values);

            % % plot exploration
            % info.plot = visualize_exploration(scenario, info.tree, info.plot);
        end
    end
end