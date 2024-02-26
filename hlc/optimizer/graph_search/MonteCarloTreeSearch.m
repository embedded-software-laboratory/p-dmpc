classdef MonteCarloTreeSearch < OptimizerInterface

    properties
        set_up_constraints (1, 1) function_handle = @()[];
        are_constraints_satisfied (1, 1) function_handle = @()[];
        random_numbers (1, :) double = [];
        rand_stream (1, 1) RandStream = RandStream('mt19937ar', Seed = 42);
    end

    methods

        function obj = MonteCarloTreeSearch()
            obj = obj@OptimizerInterface();
            obj.rand_stream = RandStream('mt19937ar', Seed = 42);
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
            obj.random_numbers = rand(obj.rand_stream, 1, n_expansions_max + Hp);
            n_successor_trims_max = mpa.maximum_branching_factor();
            % initialize variable to store control results
            info = ControlResultsInfo(iter.amount, Hp);

            all_successor_trims = mpa.successor_trims;

            % Create tree with root node
            trim = iter.trim_indices;
            root_successor_trims = all_successor_trims{trim, 1};

            tree = MonteCarloTree(1, n_successor_trims_max, n_expansions_max + Hp);
            trims = zeros(1, n_expansions_max, 'uint8');
            parents = zeros(1, n_expansions_max, 'uint32');
            children = zeros(n_successor_trims_max, n_expansions_max, 'uint32');

            root_pose = iter.x0(1:3)';
            tree.add_root(root_pose, trim, root_successor_trims);
            trims(1) = trim;
            parents(1) = 0;
            children(1:size(root_successor_trims, 2), 1) = 1;
            n_nodes = 1;

            valid_nodes_at_hp = PriorityQueue();
            shapes_tmp = cell(iter.amount, 0);

            [vehicle_obstacles, hdv_obstacles, lanelet_boundary] ...
                = obj.set_up_constraints(iter, Hp);

            iVeh = 1;
            n_expansions = 0;
            is_finished = false;

            reference_trajectory_points = squeeze(iter.reference_trajectory_points(iVeh, :, 1:2))';
            maneuvers = mpa.maneuvers;

            while (n_expansions < n_expansions_max) && ~is_finished
                % expand randomly for Hp steps
                node_id = 1;
                solution_cost = 0;
                node_pose = root_pose;

                for i_step = 1:Hp
                    is_valid = false;
                    n_expansions = n_expansions + 1;

                    % Select node to expand randomly
                    trim_positions = find(children(:, node_id));
                    n_trims = numel(trim_positions);

                    if n_trims ~= 0
                        % choose successor trim randomly
                        child_position = trim_positions(ceil(obj.random_numbers(n_expansions) * n_trims));
                    else

                        if node_id ~= 1
                            % remove edge to node without children
                            parent_id = parents(node_id);
                            children(children(parent_id) == node_id, parent_id) = 0;
                            break
                        else
                            is_finished = true;
                            break
                        end

                    end

                    % Expand node
                    parent_trim = trims(node_id);
                    successor_trims = all_successor_trims{parent_trim, i_step};
                    goal_trim = successor_trims(child_position);

                    maneuver = maneuvers{parent_trim, goal_trim};

                    c = cos(node_pose(3));
                    s = sin(node_pose(3));

                    transform = [c, -s, 0;
                                 s, c, 0;
                                 0, 0, 1];

                    start_pose = node_pose;
                    node_pose = node_pose + transform * maneuver.dpose;

                    % Cost to come
                    % Distance to reference trajectory points squared to conform with
                    % J = (x-x_ref)' Q (x-x_ref)
                    solution_cost = solution_cost + norm(node_pose(1:2) - reference_trajectory_points(:, i_step))^2;

                    is_expanded = children(child_position, node_id) ~= 1;

                    if is_expanded
                        node_id = children(child_position, node_id);
                        continue
                    end

                    node_parent = node_id;

                    shapes_without_offset = {transform(1:2, 1:2) * maneuver.area_without_offset + start_pose(1:2)};
                    shapes = {transform(1:2, 1:2) * maneuver.area + start_pose(1:2)};

                    if (i_step ~= Hp)
                        shapes_for_boundary_check = shapes_without_offset;
                        child_successor_trims = all_successor_trims{goal_trim, i_step + 1};
                    else
                        shapes_for_boundary_check = {transform(1:2, 1:2) * maneuver.area_large_offset + start_pose(1:2)};
                        child_successor_trims = [];
                    end

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
                        % remove edge
                        children(child_position, node_parent) = 0;
                        break
                    else
                        % add node
                        n_nodes = n_nodes + 1;
                        parents(1, n_nodes) = node_parent;
                        trims(:, n_nodes) = goal_trim;
                        children(1:size(child_successor_trims, 2), n_nodes) = 1;
                        children(child_position, node_parent) = n_nodes;
                        shapes_tmp(:, n_nodes) = shapes;
                        node_id = n_nodes;
                    end

                end

                if is_valid
                    valid_nodes_at_hp.push(double(node_id), double(solution_cost));
                    % Avoid double exploration
                    children(child_position, node_parent) = 0;
                end

            end

            [best_node_id, cost] = valid_nodes_at_hp.pop();

            info.n_expanded = n_expansions;

            % check if valid path exists
            if (best_node_id == -1)
                info.is_exhausted = true;
                return
            end

            tree.trim = trims;
            tree.parent = parents;
            tree.children = children;

            % set final path
            final_nodes = fliplr(tree.path_to_root(best_node_id));
            pose = zeros(3, length(final_nodes));
            pose(:, 1) = root_pose;

            for i = 2:length(final_nodes)
                start_trim = tree.trim(1, final_nodes(i - 1));
                goal_trim = tree.trim(1, final_nodes(i));

                maneuver = maneuvers{start_trim, goal_trim};

                c = cos(pose(3, i - 1));
                s = sin(pose(3, i - 1));

                transform = [c, -s, 0;
                             s, c, 0;
                             0, 0, 1];
                pose(:, i) = pose(:, i - 1) + transform * maneuver.dpose;
            end

            tree.set_final_path(cost, pose, final_nodes);

            % return cheapest path
            info.y_predicted = return_path_to(best_node_id, tree);
            info.shapes = return_path_area(shapes_tmp, tree, best_node_id);
            info.tree_path = final_nodes;
            % Predicted trims in the future Hp time steps. The first entry is the current trims
            info.predicted_trims = squeeze(double([tree.trim(1, info.tree_path(2:end))]));
            info.is_exhausted = false;
            info.needs_fallback = false;
            info.tree = tree;

        end

    end

end
