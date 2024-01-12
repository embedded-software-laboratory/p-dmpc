classdef MonteCarloTree < Tree

    properties (SetAccess = public)
        pose (3, :, :) double;
        trim (1, :, :) uint8;
        cost (1, :) single;
        children (:, :) uint32; % 0: impossible; 1: possible, unexpanded; else: child id
        successor_trims (:, :, :) uint8;
        n_nodes (1, 1) uint32 = 0;
    end

    methods

        function obj = MonteCarloTree(n_vehicles, n_successor_trims_max, n_expansions_max)

            arguments
                n_vehicles (1, 1) uint8 = 1;
                n_successor_trims_max (1, 1) uint8 = 0;
                n_expansions_max (1, 1) uint32 = 0;
            end

            obj.pose = zeros(3, n_vehicles, n_expansions_max, 'double');
            obj.trim = zeros(1, n_vehicles, n_expansions_max, 'uint8');
            obj.cost = zeros(1, n_expansions_max, 'single');
            obj.parent = zeros(1, n_expansions_max, 'uint32');
            obj.children = zeros(n_successor_trims_max^n_vehicles, n_expansions_max, 'uint32');
            obj.successor_trims = zeros(n_successor_trims_max, n_vehicles, n_expansions_max, 'uint8');
        end

        function add_root(obj, pose, trim, cost, successor_trims)
            i_node = 1;
            obj.pose(:, :, i_node) = pose;
            obj.trim(:, :, i_node) = trim;
            obj.cost(1, i_node) = cost;
            obj.parent(1, i_node) = 0;
            obj.successor_trims(1:size(successor_trims, 2), :, i_node) = successor_trims;
            obj.children(1:size(successor_trims, 2), i_node) = 1;
            obj.n_nodes = 1;
        end

        function i_node = add_node(obj, pose, trim, cost, parent, successor_trims)
            i_node = size(obj) + 1;
            obj.pose(:, :, i_node) = pose;
            obj.trim(:, :, i_node) = trim;
            obj.cost(1, i_node) = cost;
            obj.parent(1, i_node) = parent;
            obj.successor_trims(1:size(successor_trims, 2), :, i_node) = successor_trims;
            obj.children(1:size(successor_trims, 2), i_node) = 1;
            child_position = obj.successor_trims(:, :, parent) == trim;
            obj.children(child_position, parent) = i_node;
            obj.n_nodes = obj.n_nodes + 1;
        end

        function result = size(obj)
            result = obj.n_nodes;
        end

        function result = get_node(obj, i_node)
            result = node( ...
                0, ...
                obj.trim(1, :, i_node), ...
                obj.pose(1, :, i_node), ...
                obj.pose(2, :, i_node), ...
                obj.pose(3, :, i_node), ...
                obj.cost(1, i_node), ...
                obj.cost(1, i_node) ...
            );
        end

        function remove_edge(obj, i_node, trim)
            child_position = obj.successor_trims(:, :, i_node) == trim;
            obj.successor_trims(child_position, :, i_node) = 0;
            obj.children(child_position, i_node) = 0;

            if (nnz(obj.children(:, i_node)) == 0) && (i_node ~= 1)
                obj.remove_node(i_node);
            end

        end

        function remove_node(obj, i_node)
            obj.remove_edge(obj.parent(1, i_node), obj.trim(1, :, i_node));
        end

        function result = number_of_vehicles(obj)
            result = size(obj.trim, 2);
        end

        function trim = get_trim(obj, i_vehicle, i_node)
            trim = double(obj.trim(1, i_vehicle, i_node));
        end

        function x = get_x(obj, i_vehicle, i_node)
            x = obj.pose(1, i_vehicle, i_node);
        end

        function y = get_y(obj, i_vehicle, i_node)
            y = obj.pose(2, i_vehicle, i_node);
        end

        function yaw = get_yaw(obj, i_vehicle, i_node)
            yaw = obj.pose(3, i_vehicle, i_node);
        end

        function cost = get_cost(obj, i_node)
            cost = obj.cost(1, i_node);
        end

    end

end
