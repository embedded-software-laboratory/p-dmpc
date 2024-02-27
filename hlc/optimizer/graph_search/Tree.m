classdef Tree < handle

    properties (SetAccess = public)
        parent (1, :) uint32; % Index of the parent node
        % Arrays are of size n_vehicles x n_nodes
        x
        y
        yaw
        trim
        k
        g
        h
    end

    methods

        function obj = Tree(x, y, yaw, trim, k, g, h)
            obj.parent = 0;

            if nargin < 1
                return
            end

            if isa(x, 'Tree')
                % Copy constructor
                obj = x;
            else
                % New object with only root content
                obj.x = x;
                obj.y = y;
                obj.yaw = yaw;
                obj.trim = trim;
                obj.k = k;
                obj.g = g;
                obj.h = h;
            end

        end

        function i_node = get_parent(obj, i_node)
            i_node = obj.parent(i_node);
        end

        function result = path_to_root(obj, i_node)
            %% PATH_TO_ROOT  Path from node i_node to the Tree root.
            result = i_node;

            while result(end) ~= 1
                result(end + 1) = obj.parent(result(end)); %#ok<AGROW>
            end

        end

        function i_node = add_nodes(obj, parent, x, y, yaw, trim, k, g, h)
            assert( ...
                parent > 0 && parent <= numel(obj.parent) ...
                , 'Parent index %d out of bounds.\n' ...
                , parent ...
            );

            i_node = (size(obj) + 1:size(obj) + size(x, 2));
            obj.x(:, i_node) = x;
            obj.y(:, i_node) = y;
            obj.yaw(:, i_node) = yaw;
            obj.trim(:, i_node) = trim;
            obj.k(:, i_node) = k;
            obj.g(:, i_node) = g;
            obj.h(:, i_node) = h;
            obj.parent(1, i_node) = parent;
        end

        function i_node = add_node(obj, parent, node)
            assert( ...
                parent > 0 && parent <= numel(obj.parent) ...
                , 'Parent index %d out of bounds.\n' ...
                , parent ...
            );

            i_node = size(obj) + 1;
            obj.x(:, i_node) = node(:, NodeInfo.x);
            obj.y(:, i_node) = node(:, NodeInfo.y);
            obj.yaw(:, i_node) = node(:, NodeInfo.yaw);
            obj.trim(:, i_node) = node(:, NodeInfo.trim);
            obj.k(:, i_node) = node(1, NodeInfo.k);
            obj.g(:, i_node) = node(1, NodeInfo.g);
            obj.h(:, i_node) = node(1, NodeInfo.h);
            obj.parent(1, i_node) = parent;
        end

        function result = get_node(obj, i_node)
            result = node( ...
                obj.k(:, i_node), ...
                obj.trim(:, i_node), ...
                obj.x(:, i_node), ...
                obj.y(:, i_node), ...
                obj.yaw(:, i_node), ...
                obj.g(:, i_node), ...
                obj.h(:, i_node) ...
            );
        end

        function result = size(obj)
            result = size(obj.parent, 2);
        end

        function result = number_of_vehicles(obj)
            result = size(obj.x, 1);
        end

        function trim = get_trim(obj, i_vehicle, i_node)
            trim = obj.trim(i_vehicle, i_node);
        end

        function x = get_x(obj, i_vehicle, i_node)
            x = obj.x(i_vehicle, i_node);
        end

        function y = get_y(obj, i_vehicle, i_node)
            y = obj.y(i_vehicle, i_node);
        end

        function yaw = get_yaw(obj, i_vehicle, i_node)
            yaw = obj.yaw(i_vehicle, i_node);
        end

        function cost = get_cost(obj, i_node)
            cost = sum(obj.g(:, i_node));
        end

    end

end
