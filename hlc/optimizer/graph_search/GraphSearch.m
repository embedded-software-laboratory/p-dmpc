classdef GraphSearch < OptimizerInterface

    methods

        function obj = GraphSearch(scenario, mpa)
            obj = obj@OptimizerInterface(scenario, mpa);
        end

        function [info_v, graph_search_time] = run_optimizer(obj, iter, ~)
            graph_search_timer = tic;
            % execute sub controller for 1-veh scenario
            info_v = obj.do_graph_search(iter);
            graph_search_time = toc(graph_search_timer);
        end

    end

    methods (Access = private)

        function info = do_graph_search(obj, iter)
            % GRAPH_SEARCH  Expand search tree beginning at current node for Hp steps.
            %
            % OUTPUT:
            %   is_exhausted: (true/false) whether graph search is exhausted or not
            %
            Hp = obj.scenario.options.Hp;
            % initialize variable to store control results
            info = ControlResultsInfo(iter.amount, Hp, iter.vehicle_ids);

            shapes_tmp = cell(iter.amount, 0);
            % Create tree with root node
            x = iter.x0(:, 1);
            y = iter.x0(:, 2);
            yaw = iter.x0(:, 3);
            trim = iter.trim_indices;
            k = 0;
            g = 0;
            h = 0;
            info.tree = Tree(x, y, yaw, trim, k, g, h);

            % Array storing ids of nodes that may be expanded

            pq = PriorityQueue();
            pq.push(1, 0);

            if obj.scenario.options.is_allow_non_convex && iter.amount == 1
                % currently two methods for intersecting check are available:
                % 1. separating axis theorem (SAT) works only for convex polygons
                % 2. InterX: works for both convex and non-convex polygons
                method = 'InterX';
                % if 'InterX' is used, all obstacles can be vectorized to speed up the collision checking
                [vehicle_obstacles, hdv_obstacles, lanelet_boundary, lanelet_crossing_areas] = vectorize_all_obstacles(iter, obj.scenario);
            else
                method = 'sat';
                % vectorization is currently not supported for 'sat'
                vehicle_obstacles = [];
                hdv_obstacles = [];
                lanelet_boundary = [];
                lanelet_crossing_areas = [];
            end

            % Expand leaves of tree until depth or target is reached or until there
            % are no leaves
            while true
                % Select cheapest node for expansion and remove it
                cur_node_id = pq.pop();

                if (cur_node_id == -1)
                    info.n_expanded = info.tree.size();
                    info.is_exhausted = true;
                    break
                end

                % Eval edge
                [is_valid, shapes] = eval_edge_exact(iter, obj.scenario, obj.mpa, info.tree, cur_node_id, vehicle_obstacles, hdv_obstacles, lanelet_boundary, lanelet_crossing_areas, method); % two methods: 'sat' or 'InterX'

                if ~is_valid
                    % could remove node from tree here
                    %             plot_options = struct('Color',[0.4940 0.1840 0.5560],'LineWidth',0.30);
                    %             plot_obstacles(shapes,plot_options) % visualize the invalid shape
                    continue
                    %         else
                    %             plot_options = struct('Color','r','LineWidth',0.75);
                    %             plot_obstacles(shapes,plot_options) % visualize the valid shape
                end

                shapes_tmp(:, cur_node_id) = shapes;

                if info.tree.k(cur_node_id) == Hp
                    y_pred = return_path_to(cur_node_id, info.tree, obj.mpa);
                    info.y_predicted = y_pred;
                    info.shapes = return_path_area(shapes_tmp, info.tree, cur_node_id);
                    info.tree_path = fliplr(path_to_root(info.tree, cur_node_id));
                    % Predicted trims in the future Hp time steps. The first entry is the current trims
                    info.predicted_trims = info.tree.trim(:, info.tree_path);
                    info.is_exhausted = false;
                    info.needs_fallback = false;
                    info.n_expanded = info.tree.size();
                    break
                else
                    % Expand chosen node
                    new_open_nodes = expand_node( ...
                        obj.scenario ...
                        , obj.mpa ...
                        , iter ...
                        , cur_node_id ...
                        , info ...
                    );
                    g_weight = 1;
                    h_weight = 1;
                    new_open_values = info.tree.g(new_open_nodes) * g_weight + info.tree.h(new_open_nodes) * h_weight;
                    % add child nodes
                    pq.push(new_open_nodes, new_open_values);
                end

            end

        end

    end

end
