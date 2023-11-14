classdef MonteCarloTreeSearch < OptimizerInterface

    methods

        function obj = MonteCarloTreeSearch()
            obj = obj@OptimizerInterface();
        end

        function info_v = run_optimizer(obj, ~, iter, mpa, ~)
            % execute sub controller for 1-veh scenario
            info_v = obj.do_graph_search(iter, mpa);
        end

    end

    methods (Access = private)

        function info = do_graph_search(obj, iter, mpa)
            % DO_GRAPH_SEARCH  Execute Monte Carlo Tree Search.
            %
            % INPUT:
            %   iter: Iteration object.
            %
            % OUTPUT:
            %   info: ControlResultsInfo object.
            %
            assert(iter.amount == 1);
            Hp = size(iter.v_ref, 2);
            n_expansions_max = 1000;
            n_successor_trims_max = mpa.maximum_branching_factor();
            % initialize variable to store control results
            info = ControlResultsInfo(iter.amount, Hp, iter.vehicle_ids);

            % Create tree with root node
            trim = iter.trim_indices;
            successor_trims = find(mpa.transition_matrix_single(trim, :, 1));
            info.tree = MonteCarloTree(1, n_successor_trims_max, n_expansions_max);
            info.tree.add_root(iter.x0(1:3), trim, 0, 0, successor_trims);

            valid_nodes_at_hp = PriorityQueue();
            shapes_tmp = cell(iter.amount, 0);

            [vehicle_obstacles, hdv_obstacles, lanelet_boundary, lanelet_crossing_areas] = vectorize_all_obstacles(iter, Hp);

            iVeh = 1;
            n_expansions = 0;
            is_finished = false;

            while (n_expansions < n_expansions_max) && ~is_finished
                % expand randomly for Hp steps
                node_id = 1;

                for i_step = 1:Hp
                    is_valid = false;
                    n_expansions = n_expansions + 1;
                    % Select node to expand randomly
                    goal_trim = obj.random_trim(node_id, info.tree);

                    if goal_trim == 0
                        is_finished = true;
                        break
                    end

                    child_position = info.tree.successor_trims(:, :, node_id) == goal_trim;
                    is_expanded = info.tree.children(child_position, node_id) ~= 1;

                    if is_expanded
                        node_id = info.tree.children(child_position, node_id);
                        continue
                    end

                    [node_pose, node_cost, ...
                         shapes, shapes_without_offset, shapes_with_large_offset] = obj.expand_node( ...
                        goal_trim ...
                        , squeeze(iter.reference_trajectory_points(iVeh, i_step, 1:2)) ...
                        , node_id ...
                        , info.tree ...
                        , mpa ...
                    );

                    node_trim = goal_trim;
                    node_parent = node_id;

                    if (i_step ~= Hp)
                        shapes_for_boundary_check = shapes_without_offset;

                        node_successor_trims = find(mpa.transition_matrix_single(goal_trim, :, i_step + 1));
                    else
                        shapes_for_boundary_check = shapes_with_large_offset;
                    end

                    is_valid = obj.eval_edge_exact( ...
                        shapes ...
                        , shapes_without_offset ...
                        , shapes_for_boundary_check ...
                        , vehicle_obstacles{i_step} ...
                        , hdv_obstacles{i_step} ...
                        , lanelet_boundary ...
                        , lanelet_crossing_areas ...
                    );

                    if ~is_valid
                        info.tree.remove_edge(node_parent, goal_trim);
                        break
                    else
                        node_id = info.tree.add_node(node_pose, node_trim, node_cost, node_parent, node_successor_trims);
                    end

                    shapes_tmp(:, node_id) = shapes;

                end

                if is_valid
                    valid_nodes_at_hp.push(double(node_id), double(node_cost));
                    % Avoid double exploration
                    info.tree.remove_edge(node_parent, goal_trim);
                end

            end

            best_node_id = valid_nodes_at_hp.pop();

            info.n_expanded = n_expansions;

            % check if valid path exists
            if (best_node_id == -1)
                info.is_exhausted = true;
                return
            end

            % return cheapest path
            info.y_predicted = return_path_to(best_node_id, info.tree, mpa);
            info.shapes = return_path_area(shapes_tmp, info.tree, best_node_id);
            info.tree_path = fliplr(path_to_root(info.tree, best_node_id));
            % Predicted trims in the future Hp time steps. The first entry is the current trims
            info.predicted_trims = double([info.tree.trim(1, :, info.tree_path)]);
            info.is_exhausted = false;
            info.needs_fallback = false;

        end

        function trim = random_trim(~, node_id, tree)
            % RANDOM_TRIM  Return random successor trim for given node.
            %
            % INPUT:
            %   node_id: Id of node to return random successor trim for.
            %   tree: Tree to expand a node in.
            %
            % OUTPUT:
            %   trim: Random successor trim of node. If no successor trim is available, return 0.
            %
            trim_positions = find(tree.successor_trims(:, :, node_id));
            n_trims = numel(trim_positions);

            if n_trims == 0
                trim = 0;
            else
                % choose successor trim randomly
                trim = tree.successor_trims(trim_positions(randi(n_trims)), :, node_id);
            end

        end

        function [goal_pose, goal_cost, shapes, shapes_without_offset, shapes_with_large_offset] = expand_node(~ ...
                , goal_trim ...
                , reference_trajectory_point ...
                , node_id ...
                , tree ...
                , mpa ...
            )
            start_trim = tree.trim(1, :, node_id);

            maneuver = mpa.maneuvers{start_trim, goal_trim};

            c = cos(tree.pose(3, :, node_id));
            s = sin(tree.pose(3, :, node_id));

            transform = [c, -s, 0;
                         s, c, 0;
                         0, 0, 1];
            dpose = [maneuver.dx; maneuver.dy; maneuver.dyaw];
            goal_pose = transform * dpose + tree.pose(:, :, node_id);

            % Cost to come
            % Distance to reference trajectory points squared to conform with
            % J = (x-x_ref)' Q (x-x_ref)
            goal_cost = tree.cost(1, node_id) + norm( ...
                goal_pose(1:2) - reference_trajectory_point ...
            )^2;

            iVeh = 1;
            shapes = cell(iVeh, 1);
            shapes_without_offset = cell(iVeh, 1);
            shapes_with_large_offset = cell(iVeh, 1);
            shapes{iVeh} = transform(1:2, 1:2) * maneuver.area + tree.pose(1:2, :, node_id);

            shapes_without_offset{iVeh} = transform(1:2, 1:2) * maneuver.area_without_offset + tree.pose(1:2, :, node_id);

            shapes_with_large_offset{iVeh} = transform(1:2, 1:2) * maneuver.area_large_offset + tree.pose(1:2, :, node_id);

        end

        function is_valid = eval_edge_exact(~ ...
                , shapes ...
                , shapes_without_offset ...
                , shapes_for_boundary_check ...
                , vehicle_obstacles ...
                , hdv_obstacles ...
                , lanelet_boundary ...
                , lanelet_crossing_areas ...
            )
            iVeh = 1;
            is_valid = true;
            % Note1: Shape must be closed!
            % Note2: The collision check order is important.
            % Normally, check collision with lanelet boundary last would be better.
            if InterX(shapes{iVeh}, vehicle_obstacles)
                % check collision with vehicle obstacles
                is_valid = false;
                return
            end

            if (~isempty(lanelet_crossing_areas) ...
                    && InterX(shapes_without_offset{iVeh}, lanelet_crossing_areas) ...
                )
                % check collision with crossing area of lanelets
                is_valid = false;
                return
            end

            is_hdv_obstacle = ~all(all(isnan(hdv_obstacles)));

            if (is_hdv_obstacle ...
                    && InterX(shapes{iVeh}, hdv_obstacles) ...
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

        end

    end

end
