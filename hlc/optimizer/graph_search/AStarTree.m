classdef AStarTree < Tree
    %% AStarTree A class implementing a Tree data structure.
    %
    % This class implements a Tree data structure. Each node can
    % have one parent, and data in seven arrays x,y,yaw,trim,k,g,h
    %
    % Nodes are accessed through their index. The index of a node is
    % returned when it is added to the AStarTree, and corresponds to the
    % order of addition.

    properties (SetAccess = public)
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

        % CONSTRUCTOR

        function [obj, root_ID] = AStarTree(x, y, yaw, trim, k, g, h)
            %% AStarTree  Construct a new AStarTree
            %
            % t = AStarTree(another_AStarTree) is the copy-constructor for this
            % class. It returns a new AStarTree where the node order and content
            % is duplicated from the AStarTree argument.
            %
            % t = AStarTree(x,y,yaw,trim,k,g,h)
            % initialize a new AStarTree with the given values as nodes
            obj.parent = 0;

            if nargin < 1
                root_ID = 1;
                return
            end

            if isa(x, 'AStarTree')
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
                root_ID = 1;
            end

        end

        % METHODS

        function IDs = add_nodes(obj, parent, x, y, yaw, trim, k, g, h)
            %% ADD_NODES attach multiple new nodes to a parent node
            % AStarTree = AStarTree.ADD_NODES(parent_index, data) create a new node
            % with content 'data', and attach it as a child of the node
            % with index 'parent_index'. Return the modified AStarTree.
            %
            % [ AStarTree, ID ] = AStarTree.ADD_NODES(...) returns the modified AStarTree and
            % the index of the newly created node.

            assert( ...
                parent > 0 && parent <= numel(obj.parent) ...
                , 'Parent index %d out of bounds.\n' ...
                , parent ...
            );

            IDs = (size(obj) + 1:size(obj) + size(x, 2));
            obj.x(:, IDs) = x;
            obj.y(:, IDs) = y;
            obj.yaw(:, IDs) = yaw;
            obj.trim(:, IDs) = trim;
            obj.k(:, IDs) = k;
            obj.g(:, IDs) = g;
            obj.h(:, IDs) = h;
            obj.parent(1, IDs) = parent;
        end

        function IDs = add_node(obj, parent, node)
            %% ADD_NODES attach multiple new nodes to a parent node
            % AStarTree = AStarTree.ADD_NODE(parent_index, data) create a new node
            % with content 'data', and attach it as a child of the node
            % with index 'parent_index'. Return the modified AStarTree.
            %
            % [ AStarTree, ID ] = AStarTree.ADD_NODE(...) returns the modified AStarTree and
            % the index of the newly created node.

            assert( ...
                parent > 0 && parent <= numel(obj.parent) ...
                , 'Parent index %d out of bounds.\n' ...
                , parent ...
            );

            IDs = size(obj) + 1;
            obj.x(:, IDs) = node(:, NodeInfo.x);
            obj.y(:, IDs) = node(:, NodeInfo.y);
            obj.yaw(:, IDs) = node(:, NodeInfo.yaw);
            obj.trim(:, IDs) = node(:, NodeInfo.trim);
            obj.k(:, IDs) = node(1, NodeInfo.k);
            obj.g(:, IDs) = node(1, NodeInfo.g);
            obj.h(:, IDs) = node(1, NodeInfo.h);
            obj.parent(1, IDs) = parent;
        end

        function result = get_node(obj, ID)
            % GET_NODE Return node matrix of desired index ID
            result = node( ...
                obj.k(:, ID), ...
                obj.trim(:, ID), ...
                obj.x(:, ID), ...
                obj.y(:, ID), ...
                obj.yaw(:, ID), ...
                obj.g(:, ID), ...
                obj.h(:, ID) ...
            );
        end

        function result = size(obj)
            %% SIZE Return the number of nodes in the Tree.
            result = size(obj.x, 2);
        end

        function result = number_of_vehicles(obj)
            %% NUMBER_OF_VEHICLES  Return the number of vehicles in the Tree.
            result = size(obj.x, 1);
        end

        function trim = get_trim(obj, i_vehicle, ID)
            %% GET_TRIM  Return the trim of the given node.
            trim = obj.trim(i_vehicle, ID);
        end

        function x = get_x(obj, i_vehicle, ID)
            %% GET_X  Return the x coordinate of the given node.
            x = obj.x(i_vehicle, ID);
        end

        function y = get_y(obj, i_vehicle, ID)
            %% GET_X  Return the y coordinate of the given node.
            y = obj.y(i_vehicle, ID);
        end

        function yaw = get_yaw(obj, i_vehicle, ID)
            %% GET_X  Return the yaw coordinate of the given node.
            yaw = obj.yaw(i_vehicle, ID);
        end

    end

end
