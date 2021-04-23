function [u, y_pred, info] = graph_search(scenario, iter, prev_info)
    info = prev_info;

    % Create tree with root node
    % TODO Choose trim_indices depending on measurement
    init_depth = 0;
    g_values = zeros(scenario.nVeh,1);
    h_values = zeros(scenario.nVeh,1);
    cur_node = node(...
        init_depth, ...
        prev_info.trim_indices, ...
        iter.x0(:,1), ...
        iter.x0(:,2), ...
        iter.x0(:,3), ...
        g_values, ...
        h_values...
    );
    info.tree = tree(cur_node);
    
    % Array storing ids of nodes that may be expanded
    % TODO Preallocate
    open_nodes = 1;
    open_values = sum_values(info.tree, open_nodes);
    
    % Array storing ids of nodes that were visited
    visited_nodes = [];
    
    % Expand leaves of tree until depth or target is reached or until there 
    % are no leaves
    % while ((cur_value < min_value && (cur_node.depth < h_p)) ...
    %        || isempty(candidates))
    while true
        % Choose cheapest node for expansion
        % [cur_node_id, open_id] = get_next_node(info.tree, open_nodes);
        [open_nodes, open_values] = sort_open_list(open_nodes, open_values);
        cur_node_id = open_nodes(1);
        cur_node = info.tree.Node{cur_node_id};
        
        % Eval edges if it is a final candidate
        if cur_node.depth == h_p
            [info.tree, open_nodes, open_values, search_finished] = eval_path_exact(scenario, info.tree, open_nodes, open_values, cur_node_id);
            % open_values = sum_values(info.tree,open_nodes);
            if search_finished
                y_pred = return_path_to(cur_node_id, info.tree, scenario.combined_graph);
                % TODO u
                u = 0;
                info.tree_path = fliplr(pathtoroot(info.tree, cur_node_id));
                info.trim_indices = info.tree.Node{info.tree_path(2)}.trims;
                break
            end
        else
            % Expand chosen node
            [expanded_nodes, info.tree] = expand_node(...
                scenario...
                ,iter...
                ,info.tree...
                ,cur_node_id...
                ,visited_nodes...
            );
            % add child nodes
            open_nodes = [open_nodes, expanded_nodes];
            open_values = [open_values, sum_values(info.tree, expanded_nodes)];
            % remove expanded parent node
            open_nodes(1) = [];
            open_values(1) = [];

            % plot exploration
%             info.plot = visualize_exploration(scenario, info.tree, info.plot);
        end
    end 
end