function [u, y_pred, info] = graph_search(scenario, iter)
% GRAPH_SEARCH  Expand search tree beginning at current node for Hp steps.
    
%     plot_shapes(scenario.dynamic_obstacle_area); % visualize other vehicles' predicted occupation areas
%     plot_shapes(scenario.dynamic_obstacle_reachableSets); % visualize other vehicles' reachable sets
    info = struct;
    shapes_tmp = cell(scenario.nVeh,0);
    % Create tree with root node
    x = iter.x0(:,1);
    y = iter.x0(:,2);
    yaw = iter.x0(:,3);
    trim = iter.trim_indices;
    k = 0;
    g = 0;
    h = 0;
    info.tree = Tree(x,y,yaw,trim,k,g,h);

    info_prev = iter.info_prev; % information from previous time step

    info.n_exhausted = 0; % number of times that the graph search is exhausted    

    % Assume that the fail-safe trajectory is invariant safe in the sense that collision can only caused 
    % by other vehicles (for example, the ego vehicle already stops but other vehicles collide with it)
    info.invariant_safe = true; % todo: check if the fail-safe trajectory is invariant safe

    
    % Array storing ids of nodes that may be expanded
    
    pq = PriorityQueue();
    pq.push(1, 0); % push initial node with cost zero

    % Expand leaves of tree until depth or target is reached or until there 
    % are no leaves
    count = 1;
    while true
        % Select cheapest node for expansion and remove it
        cur_node_id = pq.pop();

        if (cur_node_id == -1)
            warning('Graph search is exhausted. Previous fail-safe trajectory will be used.')
            info.n_exhausted = info_prev.n_exhausted + 1; % number of exhaustion times + 1
            info.exhaustReason_reachableSets = true; % if exhaustion because of the reachable sets of other vehicles

            % initialize the tree again
            % Create tree with root node
            x = iter.x0(:,1);
            y = iter.x0(:,2);
            yaw = iter.x0(:,3);
            trim = iter.trim_indices;
            k = 0;
            g = 0;
            h = 0;
            info.tree = Tree(x,y,yaw,trim,k,g,h);
            
            tree_path = [info_prev.tree_path(3:end) info_prev.tree_path(end)]; % delete the first two element but not one because the tree is already initialized with one node corresponding to the initial trim
            for iNode=1:length(tree_path) % Add new nodes. Each node corresponds to one planned trim in the future.
                idx = tree_path(iNode);
                add_nodes(info.tree, iNode, info_prev.trees{1,1}.x(idx), info_prev.trees{1,1}.y(idx),info_prev.trees{1,1}.yaw(idx),...
                    info_prev.trees{1,1}.trim(idx), info_prev.trees{1,1}.k(idx), info_prev.trees{1,1}.g(idx), info_prev.trees{1,1}.h(idx));
            end

            % use trajectory from previous time step         
            info.shapes = del_first_rpt_last(info_prev.shapes);
            info.tree_path = 1:(scenario.Hp+1); % no irrelative trims in the tree
            info.predicted_trims = del_first_rpt_last(info_prev.predicted_trims); % planned trims in the future Hp time steps
            info.trim_indices = info.predicted_trims(:,2);
            
            nTicks = scenario.tick_per_step + 1;
            y_pred = {[info_prev.y_predicted{1,1}(nTicks+1:end,:); repmat(info_prev.y_predicted{1,1}(end,:),nTicks,1)]}; % delete the first nTicks rows and repeat the last row for nTicks times
            
            u = zeros(scenario.nVeh);
            

            % if the exhaust time is greater than Hp and the fail-safe trajectory is not invariant safe
            if info.n_exhausted>=scenario.Hp && ~info.invariant_safe 
                ME = MException( ...
                    'MATLAB:graph_search:tree_exhausted' ...
                    ,'No more open nodes to explore' ...
                );
                throw(ME);
            end

            break

        end
        

        % Eval edge 
        [is_valid, shapes, is_collide_with_reachableSets] = eval_edge_exact(scenario, info.tree, cur_node_id);


        if ~is_valid
            % store what does the vehicle collide with
            if is_collide_with_reachableSets
                info.exhaustReason_reachableSets = false; % There exists 
            end
            % todo could remove node from tree here
            continue
        end
        shapes_tmp(:,cur_node_id) = shapes;
        if info.tree.k(cur_node_id) == scenario.Hp
            y_pred = return_path_to(cur_node_id, info.tree, scenario.mpa);
            u = zeros(scenario.nVeh);
            info.shapes = return_path_area(shapes_tmp, info.tree, cur_node_id);
            info.tree_path = fliplr(path_to_root(info.tree, cur_node_id));
            info.predicted_trims = info.tree.trim(:,info.tree_path); % planned trims in the future Hp time steps
            info.trim_indices = info.predicted_trims(:,2);
            info.n_exhausted = 0; % reset the number of exhaustion times

%             plot_shapes(info.shapes); % visualize the ego vehicle's predicted occupation areas
            
            break
        else
            % Expand chosen node
            new_open_nodes = expand_node(scenario,iter,cur_node_id,info);
            g_weight = 1;
            h_weight = 1;
            new_open_values = info.tree.g(new_open_nodes) * g_weight + info.tree.h(new_open_nodes) * h_weight;
            % add child nodes
            pq.push(new_open_nodes,new_open_values);

%             % plot exploration
%             info.plot = visualize_exploration(scenario, info.tree, info.plot);
        end
    end
end
