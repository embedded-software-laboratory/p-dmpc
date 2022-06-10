function [u, y_pred, info] = graph_search(scenario, iter)
% GRAPH_SEARCH  Expand search tree beginning at current node for Hp steps.

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
    
    % Array storing ids of nodes that may be expanded
    
    pq = PriorityQueue();
    pq.push(1, 0);

    if scenario.is_allow_non_convex
        % currently two methods for intersecting check are available:
        % 1. separating axis theorem (SAT) works only for convex polygons
        % 2. InterX: works for both convex and non-convex polygons
        method = 'InterX';
        % if 'InterX' is used, all obstacles can be vectorized to speed up the collision checking 
        [vehicle_obstacles, lanelet_boundary, lanelet_crossing_areas] = vectorize_all_obstacles(scenario);
    else
        method = 'sat';
        % vectorization is currently not supported for 'sat'
        vehicle_obstacles = [];
        lanelet_boundary = [];
        lanelet_crossing_areas = [];
%             method = 'InterX';
    end

    % Expand leaves of tree until depth or target is reached or until there 
    % are no leaves
    while true
        % Select cheapest node for expansion and remove it
        cur_node_id = pq.pop();
        if (cur_node_id == -1)
            ME = MException( ...
                'MATLAB:graph_search:tree_exhausted' ...
                ,'No more open nodes to explore' ...
            );
            disp(['Graph search exhausted with vehicle ID: ' num2str(scenario.vehicles.ID) ', at time step: ' num2str(scenario.k) '.'])
            throw(ME);
        end

        % Eval edge        
        [is_valid, shapes] = eval_edge_exact(scenario, info.tree, cur_node_id, vehicle_obstacles, lanelet_boundary, lanelet_crossing_areas, method); % two methods: 'sat' or 'InterX'
        
        if ~is_valid
            % could remove node from tree here
            continue
        end
        
        shapes_tmp(:,cur_node_id) = shapes;
        if info.tree.k(cur_node_id) == scenario.Hp
            y_pred = return_path_to(cur_node_id, info.tree, scenario);
            u = zeros(scenario.nVeh);
            info.shapes = return_path_area(shapes_tmp, info.tree, cur_node_id);
            info.tree_path = fliplr(path_to_root(info.tree, cur_node_id));
            % Predicted trims in the future Hp time steps. The first
            % entry is the current trims
            info.predicted_trims = info.tree.trim(:,info.tree_path); 
            info.trim_indices = info.tree.trim(:,info.tree_path(2));
            break
        else
            % Expand chosen node
            new_open_nodes = expand_node(...
                scenario...
                ,iter...
                ,cur_node_id...
                ,info...
            );
            g_weight = 1;
            h_weight = 1;
            new_open_values = info.tree.g(new_open_nodes) * g_weight + info.tree.h(new_open_nodes) * h_weight;
            % add child nodes
            pq.push(new_open_nodes,new_open_values);

            % % plot exploration
            % info.plot = visualize_exploration(scenario, info.tree, info.plot);
        end
    end
end
