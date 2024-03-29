classdef Tree < handle
    %% TREE A class implementing a Tree data structure.
    %
    % This class implements a Tree data structure. Each node can
    % have one parent, and data in seven arrays x,y,yaw,trim,k,g,h
    %
    % Nodes are accessed through their index. The index of a node is
    % returned when it is added to the Tree, and corresponds to the
    % order of addition.

    properties (SetAccess = private)
        % Index of the parent node. The root of the Tree as a parent index
        % equal to 0.
        parent = [0]; %#ok<NBRAK>
    end

    properties (SetAccess = public)
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

        function [obj, root_ID] = Tree(x, y, yaw, trim, k, g, h)
            %% TREE  Construct a new Tree
            %
            % t = TREE(another_tree) is the copy-constructor for this
            % class. It returns a new Tree where the node order and content
            % is duplicated from the Tree argument.
            %
            % t = TREE(x,y,yaw,trim,k,g,h)
            % initialize a new Tree with the given values as nodes
            if nargin < 1
                root_ID = 1;
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
                root_ID = 1;
            end

        end

        % METHODS

        function IDs = add_nodes(obj, parent, x, y, yaw, trim, k, g, h)
            %% ADD_NODES attach multiple new nodes to a parent node
            % Tree = Tree.ADD_NODES(parent_index, data) create a new node
            % with content 'data', and attach it as a child of the node
            % with index 'parent_index'. Return the modified Tree.
            %
            % [ Tree, ID ] = Tree.ADD_NODES(...) returns the modified Tree and
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
            obj.parent(IDs, 1) = parent;
        end

        function IDs = add_node(obj, parent, node)
            %% ADD_NODES attach multiple new nodes to a parent node
            % Tree = Tree.ADD_NODE(parent_index, data) create a new node
            % with content 'data', and attach it as a child of the node
            % with index 'parent_index'. Return the modified Tree.
            %
            % [ Tree, ID ] = Tree.ADD_NODE(...) returns the modified Tree and
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
            obj.parent(IDs, 1) = parent;
        end

        function flag = is_leaf(obj, ID)
            %% IS_LEAF  Return true if given ID matches a leaf node.
            % A leaf node is a node that has no children.
            assert( ...
                ID >= 1 && ID <= numel(obj.parent) ...
                , 'No node with ID %d.' ...
                , ID ...
            );

            flag = ~any(obj.parent == ID);
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

        function ID = get_parent(obj, ID)
            %% GET_PARENT  Return the ID of the parent of the given node.
            assert( ...
                ID >= 1 && ID <= numel(obj.parent) ...
                , 'No node with ID %d.' ...
                , ID ...
            );

            ID = obj.parent(ID);
        end

        function result = path_to_root(obj, ID)
            %% PATH_TO_ROOT  Path from node ID to the Tree root.
            result = ID;

            while result(end) ~= 1
                result(end + 1) = obj.parent(result(end)); %#ok<AGROW>
            end

        end

    end

end
