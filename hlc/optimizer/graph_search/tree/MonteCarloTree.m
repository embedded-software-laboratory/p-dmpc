classdef MonteCarloTree < Tree

    properties (SetAccess = public)
        root_pose (1, 3) double;

        trim (1, :) uint8;
        children (:, :) uint32; % 0: impossible; 1: possible, unexpanded; else: child id

        % remember for final path
        final_cost (1, 1) single;
        final_pose (3, :) double;
        final_nodes (1, :) uint32;

        n_nodes (1, 1) uint32 = 0;
    end

    methods

        function obj = MonteCarloTree(n_vehicles, n_successor_trims_max, n_expansions_max)

            arguments
                n_vehicles (1, 1) uint8 = 1;
                n_successor_trims_max (1, 1) uint8 = 0;
                n_expansions_max (1, 1) uint32 = 0;
            end

            assert(n_vehicles == 1);
            obj.trim = zeros(1, n_expansions_max, 'uint8');
            obj.parent = zeros(1, n_expansions_max, 'uint32');
            obj.children = zeros(n_successor_trims_max^n_vehicles, n_expansions_max, 'uint32');
        end

        function add_root(obj, pose, trim, successor_trims)
            i_node = 1;
            obj.root_pose = pose;
            obj.trim(:, i_node) = trim;
            obj.parent(1, i_node) = 0;
            obj.children(1:size(successor_trims, 2), i_node) = 1;
            obj.n_nodes = 1;
        end

        function i_node = add_node(obj, trim, parent, successor_trims, ith_trim)
            i_node = size(obj) + 1;
            obj.parent(1, i_node) = parent;
            obj.trim(:, i_node) = trim;
            obj.children(1:size(successor_trims, 2), i_node) = 1;
            obj.children(ith_trim, parent) = i_node;
            obj.n_nodes = obj.n_nodes + 1;
        end

        function result = size(obj)
            result = obj.n_nodes;
        end

        function remove_edge(obj, i_node, child_position)
            obj.children(child_position, i_node) = 0;

            if (nnz(obj.children(:, i_node)) == 0) && (i_node ~= 1)
                obj.remove_node(i_node);
            end

        end

        function remove_node(obj, i_node)
            obj.remove_edge( ...
                obj.parent(1, i_node), ...
                find(obj.children(:, obj.parent(1, i_node)) == i_node) ...
            );
        end

        function result = number_of_vehicles(obj)
            result = size(obj.trim, 1);
        end

        function trim = get_trim(obj, ~, i_node)
            trim = double(obj.trim(1, i_node));
        end

        function set_final_path( ...
                obj ...
                , final_cost ...
                , final_pose ...
                , final_nodes ...
            )
            obj.final_cost = final_cost;
            obj.final_pose = final_pose;
            obj.final_nodes = final_nodes;
        end

        function value = pose(obj, trims, d_pose_primitives)
            % TODO remove if unused
            x = obj.root_pose(1);
            y = obj.root_pose(2);
            yaw = obj.root_pose(3);
            transformation = [
                              cos(yaw), -sin(yaw), x;
                              sin(yaw), cos(yaw), y;
                              0, 0, 1;
                              ];
            yaw_total = yaw;

            for the_trim = trims
                x = d_pose_primitives(1, the_trim);
                y = d_pose_primitives(2, the_trim);
                yaw = d_pose_primitives(3, the_trim);

                transformation = transformation * [
                                                   cos(yaw), -sin(yaw), x;
                                                   sin(yaw), cos(yaw), y;
                                                   0, 0, 1;
                                                   ];
                yaw_total = yaw_total + yaw;
            end

            xy = transformation * [0; 0; 1];
            value = [xy(1); xy(2); yaw_total];
        end

        function x = get_x(obj, ~, i_node)
            x = obj.final_pose(1, obj.final_nodes == i_node);
        end

        function y = get_y(obj, ~, i_node)
            y = obj.final_pose(2, obj.final_nodes == i_node);
        end

        function yaw = get_yaw(obj, ~, i_node)
            yaw = obj.final_pose(3, obj.final_nodes == i_node);
        end

        function cost = get_cost(obj, ~)
            cost = double(obj.final_cost);
        end

    end

end
