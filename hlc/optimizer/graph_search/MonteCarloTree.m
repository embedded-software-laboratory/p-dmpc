classdef MonteCarloTree < Tree

    properties (SetAccess = private)
    end

    properties (SetAccess = public)
        pose (3, :, :) double;
        trim (1, :, :) uint8;
        cost (1, :) single;
        parent (1, :) uint32;
        children (:, :) uint32; % 0: impossible; 1: possible, unexpanded; else: child id
        successor_trims (:, :, :) uint8;
        n_nodes (1, 1) uint32 = 0;
    end

    methods

        % CONSTRUCTOR

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

        % METHODS
        function add_root(obj, pose, trim, cost, parent, successor_trims)
            IDs = 1;
            obj.pose(:, :, IDs) = pose;
            obj.trim(:, :, IDs) = trim;
            obj.cost(1, IDs) = cost;
            obj.parent(1, IDs) = parent;
            obj.successor_trims(1:size(successor_trims, 2), :, IDs) = successor_trims;
            obj.children(1:size(successor_trims, 2), IDs) = 1;
            obj.n_nodes = 1;
        end

        function ID = add_node(obj, pose, trim, cost, parent, successor_trims)
            ID = size(obj) + 1;
            obj.pose(:, :, ID) = pose;
            obj.trim(:, :, ID) = trim;
            obj.cost(1, ID) = cost;
            obj.parent(1, ID) = parent;
            obj.successor_trims(1:size(successor_trims, 2), :, ID) = successor_trims;
            obj.children(1:size(successor_trims, 2), ID) = 1;
            child_position = obj.successor_trims(:, :, parent) == trim;
            obj.children(child_position, parent) = ID;
            obj.n_nodes = obj.n_nodes + 1;
        end

        function result = size(obj)
            %% SIZE Return the number of nodes in the Tree.
            % result = size(obj.parent, 2);
            result = obj.n_nodes;
        end

        function result = get_node(obj, ID)
            % GET_NODE Return node matrix of desired index ID
            result = node( ...
                0, ...
                obj.trim(1, :, ID), ...
                obj.pose(1, :, ID), ...
                obj.pose(2, :, ID), ...
                obj.pose(3, :, ID), ...
                obj.cost(1, ID), ...
                obj.cost(1, ID) ...
            );
        end

        function remove_edge(obj, node_id, trim)
            child_position = obj.successor_trims(:, :, node_id) == trim;
            obj.successor_trims(child_position, :, node_id) = 0;
            obj.children(child_position, node_id) = 0;

            if (nnz(obj.children(:, node_id)) == 0) && (node_id ~= 1)
                obj.remove_node(node_id);
            end

        end

        function remove_node(obj, node_id)
            obj.remove_edge(obj.parent(1, node_id), obj.trim(1, :, node_id));
        end

        function result = number_of_vehicles(obj)
            %% NUMBER_OF_VEHICLES  Return the number of vehicles in the Tree.
            result = size(obj.trim, 2);
        end

        function trim = get_trim(obj, i_vehicle, ID)
            %% GET_TRIM  Return the trim of the given node.
            trim = obj.trim(1, i_vehicle, ID);
        end

        function x = get_x(obj, i_vehicle, ID)
            %% GET_X  Return the x coordinate of the given node.
            x = obj.pose(1, i_vehicle, ID);
        end

        function y = get_y(obj, i_vehicle, ID)
            %% GET_X  Return the y coordinate of the given node.
            y = obj.pose(2, i_vehicle, ID);
        end

        function yaw = get_yaw(obj, i_vehicle, ID)
            %% GET_X  Return the yaw coordinate of the given node.
            yaw = obj.pose(3, i_vehicle, ID);
        end

    end

end
