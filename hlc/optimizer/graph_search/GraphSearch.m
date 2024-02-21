classdef GraphSearch < OptimizerInterface

    properties
        set_up_constraints (1, 1) function_handle = @()[];
        are_constraints_satisfied (1, 1) function_handle = @()[];
    end

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

        function info = do_graph_search(obj, iter, mpa, options)
            % GRAPH_SEARCH  Expand search tree beginning at current node for Hp steps.
            %
            % OUTPUT:
            %   is_exhausted: (true/false) whether graph search is exhausted or not

            % initialize variable to store control results
            info = ControlResultsInfo(iter.amount, options.Hp);

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

            [vehicle_obstacles, hdv_obstacles, lanelet_boundary] ...
                = obj.set_up_constraints(iter, options.Hp);

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
                [is_valid, shapes] = obj.eval_edge_exact( ...
                    iter, ...
                    options, ...
                    mpa, ...
                    info.tree, ...
                    cur_node_id, ...
                    vehicle_obstacles, ...
                    hdv_obstacles, ...
                    lanelet_boundary ...
                );

                if ~is_valid
                    continue
                end

                shapes_tmp(:, cur_node_id) = shapes;

                if info.tree.k(cur_node_id) == options.Hp
                    info.y_predicted = return_path_to(cur_node_id, info.tree);
                    info.shapes = return_path_area(shapes_tmp, info.tree, cur_node_id);
                    info.tree_path = fliplr(path_to_root(info.tree, cur_node_id));
                    % Predicted trims in the future Hp time steps. The first entry is the current trims
                    info.predicted_trims = info.tree.trim(:, info.tree_path(2:end));
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

        function [is_valid, shapes] = eval_edge_exact( ...
                obj, ...
                iter, ...
                options, ...
                mpa, ...
                tree, ...
                iNode, ...
                vehicle_obstacles, ...
                hdv_obstacles, ...
                lanelet_boundary ...
            )
            % EVAL_EDGE_EXACT   Evaluate if edge to vertex is valid.
            %
            % OUTPUT:
            %   is_valid: logical, indicates whether the selected node is collision-free
            %
            %   shapes: occupied area of the vehicle if the selected node is chosen
            %

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

                i_step = cK;

                % check if collides with other vehicles' predicted trajectory or lanelets
                is_valid = obj.are_constraints_satisfied( ...
                    iter, ...
                    iVeh, ...
                    shapes, ...
                    shapes_for_boundary_check, ...
                    i_step, ...
                    vehicle_obstacles, ...
                    lanelet_boundary, ...
                    hdv_obstacles ...
                );

                if ~is_valid
                    return;
                end

            end

        end

    end

end
