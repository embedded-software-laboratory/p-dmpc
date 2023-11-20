classdef GraphSearch < OptimizerInterface

    methods

        function obj = GraphSearch()
            obj = obj@OptimizerInterface();
        end

        function info_v = run_optimizer(obj, ~, iter, mpa, options)
            % execute sub controller for 1-veh scenario
            info_v = obj.do_graph_search(iter, mpa, options);
        end

    end

    methods (Access = private)

        function info = do_graph_search(~, iter, mpa, options)
            % GRAPH_SEARCH  Expand search tree beginning at current node for Hp steps.
            %
            % OUTPUT:
            %   is_exhausted: (true/false) whether graph search is exhausted or not

            % initialize variable to store control results
            info = ControlResultsInfo(iter.amount, options.Hp, iter.vehicle_ids);

            shapes_tmp = cell(iter.amount, 0);
            % Create tree with root node
            x = iter.x0(:, 1);
            y = iter.x0(:, 2);
            yaw = iter.x0(:, 3);
            trim = iter.trim_indices;
            k = 0;
            g = 0;
            h = 0;
            info.tree = AStarTree(x, y, yaw, trim, k, g, h);

            % Array storing ids of nodes that may be expanded

            pq = PriorityQueue();
            pq.push(1, 0);

            if options.is_allow_non_convex && iter.amount == 1
                % currently two methods for intersecting check are available:
                % 1. separating axis theorem (SAT) works only for convex polygons
                % 2. InterX: works for both convex and non-convex polygons
                method = 'InterX';
                % if 'InterX' is used, all obstacles can be vectorized to speed up the collision checking
                [vehicle_obstacles, hdv_obstacles, lanelet_boundary, lanelet_crossing_areas] = vectorize_all_obstacles(iter, options.Hp);
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
                [is_valid, shapes] = GraphSearch.eval_edge_exact(iter, options, mpa, info.tree, cur_node_id, vehicle_obstacles, hdv_obstacles, lanelet_boundary, lanelet_crossing_areas, method); % two methods: 'sat' or 'InterX'

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

                if info.tree.k(cur_node_id) == options.Hp
                    y_pred = return_path_to(cur_node_id, info.tree, mpa);
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
                        options, ...
                        mpa, ...
                        iter, ...
                        cur_node_id, ...
                        info ...
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

    methods (Static, Access = private)

        function [is_valid, shapes] = eval_edge_exact(iter, options, mpa, tree, iNode, vehicle_obstacles, hdv_obstacles, lanelet_boundary, lanelet_crossing_areas, method)
            % EVAL_EDGE_EXACT   Evaluate if step is valid.
            %
            % INPUT:
            %   options: instance of the class Config
            %
            %   tree: instance of the class Tree
            %
            %   iNode: tree node to be checked
            %
            %   method: string, method to check collision, currently two methods are available:
            %       1. 'sat': separating axis theorem
            %       2. 'InterX': an algorithm to fast check intersection of curves
            %
            % OUTPUT:
            %   is_valid: logical, indicates whether the selected node is collision-free
            %
            %   shapes: occupied area of the vehicle if the selected node is chosen
            %

            if nargin < 8
                method = 'sat'; % use the default method, separating axis theorem, to check if collide
            end

            is_valid = true;
            % maneuver shapes correspond to movement TO node
            node_id_parent = get_parent(tree, iNode);
            shapes = cell(iter.amount, 1);
            shapes_without_offset = cell(iter.amount, 1);
            shapes_for_boundary_check = cell(iter.amount, 1);

            if ~node_id_parent % root node without parent
                return;
            end

            pX = tree.x(:, node_id_parent);
            pY = tree.y(:, node_id_parent);
            pYaw = tree.yaw(:, node_id_parent);
            pTrim = tree.trim(:, node_id_parent);

            cTrim = tree.trim(:, iNode);
            cK = tree.k(:, iNode);

            for iVeh = 1:iter.amount
                t1 = pTrim(iVeh);
                t2 = cTrim(iVeh);

                maneuver = mpa.maneuvers{t1, t2};

                c = cos(pYaw(iVeh));
                s = sin(pYaw(iVeh));

                shape_x = c * maneuver.area(1, :) - s * maneuver.area(2, :) + pX(iVeh);
                shape_y = s * maneuver.area(1, :) + c * maneuver.area(2, :) + pY(iVeh);
                shapes{iVeh} = [shape_x; shape_y];

                shape_x_without_offset = c * maneuver.area_without_offset(1, :) - s * maneuver.area_without_offset(2, :) + pX(iVeh);
                shape_y_without_offset = s * maneuver.area_without_offset(1, :) + c * maneuver.area_without_offset(2, :) + pY(iVeh);
                shapes_without_offset{iVeh} = [shape_x_without_offset; shape_y_without_offset];

                if tree.k(iNode) == options.Hp
                    % with larger offset
                    shape_x_for_boundary_check = c * maneuver.area_large_offset(1, :) - s * maneuver.area_large_offset(2, :) + pX(iVeh);
                    shape_y_for_boundary_check = s * maneuver.area_large_offset(1, :) + c * maneuver.area_large_offset(2, :) + pY(iVeh);
                    shapes_for_boundary_check{iVeh} = [shape_x_for_boundary_check; shape_y_for_boundary_check];
                else
                    % without offset
                    shapes_for_boundary_check{iVeh} = shapes_without_offset{iVeh};
                end

                iStep = cK;

                switch method
                    case 'sat'
                        % check if collides with other vehicles' predicted trajectory or lanelets

                        if collision_with(iter, iVeh, shapes, shapes_for_boundary_check, iStep)
                            is_valid = false;
                            return;
                        end

                    case 'InterX'
                        assert(iter.amount == 1) % if not 1, code adaption is needed
                        % Note1: Shape must be closed!
                        % Note2: The collision check order is important.
                        % Normally, check collision with lanelet boundary last would be better.
                        if InterX(shapes{iVeh}, vehicle_obstacles{iStep})
                            % check collision with vehicle obstacles
                            is_valid = false;
                            return
                        end

                        if options.amount > 1 && InterX(shapes_without_offset{iVeh}, lanelet_crossing_areas)
                            % check collision with crossing area of lanelets
                            is_valid = false;
                            return
                        end

                        is_hdv_obstacle = ~all(all(isnan(hdv_obstacles{iStep})));

                        if (is_hdv_obstacle && ...
                                InterX(shapes{iVeh}, hdv_obstacles{iStep}) ...
                            )
                            % check collision with manual vehicle obstacles
                            is_valid = false;
                            return
                        end

                        % check collision with lanelet obstacles
                        if InterX(shapes_for_boundary_check{iVeh}, lanelet_boundary)
                            is_valid = false;
                            return
                        end

                    otherwise
                        error("Please specify one of the following mthodes to check collision: 'sat', 'InterX'.")
                end

            end

        end

    end

end
