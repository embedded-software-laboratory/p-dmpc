classdef Tree
%% TREE A class implementing a Tree data structure.
%
% This class implements a simple Tree data structure. Each node can
% have one parent, and store any kind of data.
%
% Nodes are accessed through their index. The index of a node is
% returned when it is added to the Tree, and corresponds to the
% order of addition.
%
% Internally, the class simply manage an array referencing the node parent
% indices, and a cell array containing the node data. 
    properties (SetAccess = private)
        % Index of the parent node. The root of the Tree as a parent index
        % equal to 0.
        parent = [ 0 ]; %#ok<NBRAK>
    end
    
    properties (SetAccess = public)
        node = { [] }; % Hold the data at each node
    end
    
    methods
        
        % CONSTRUCTOR
        
        function [obj, root_ID] = Tree(content)
            %% TREE  Construct a new Tree
            %
            % t = TREE(another_tree) is the copy-constructor for this
            % class. It returns a new Tree where the node order and content
            % is duplicated from the Tree argument.
            %
            % t = TREE(root_content) where 'root_content' is not a Tree,
            % initialize a new Tree with only the root node, and set its
            % content to be 'root_content'.
            if nargin < 1
                root_ID = 1;
                return
            end
            
            if isa(content, 'Tree')
                % Copy constructor
                obj = content;
            else
                % New object with only root content
                obj.node = { content };
                root_ID = 1;
            end
            
        end
        
        
        % METHODS
        
        function [obj, IDs] = add_nodes(obj, parents, datas)
            %% ADD_NODES attach multiple new nodes to a parent node
            % Tree = Tree.ADD_NODES(parent_index, data) create a new node
            % with content 'data', and attach it as a child of the node
            % with index 'parent_index'. Return the modified Tree.
            % 
            % [ Tree, ID ] = Tree.ADD_NODES(...) returns the modified Tree and
            % the index of the newly created node.
            
            assert( ...
                all(parents>0) && all(parents<=numel(obj.parent)) ...
                , 'Parent index %d out of bounds.\n' ...
                , parents ...
            );

            IDs = (numel(obj.node)+1:numel(obj.node)+numel(parents));
            obj.node(IDs,1) = datas;
            obj.parent(IDs,1) = parents;
        end
        
        function flag = is_leaf(obj, ID)
            %% IS_LEAF  Return true if given ID matches a leaf node.
            % A leaf node is a node that has no children.
            assert(...
                ID >= 1 && ID <= numel(obj.parent) ...
                ,'No node with ID %d.' ...
                , ID ...
            );

            flag = ~any( obj.parent == ID );
        end
        
        function content = get(obj, ID)
            %% GET  Return the content of the given node ID.
            content = obj.node{ID};
        end

        function obj = set(obj, ID, content)
            %% SET  Set the content of given node ID and return the modifed Tree.
            obj.node{ID} = content;
        end
        
        function ID = get_parent(obj, ID)
        %% GET_PARENT  Return the ID of the parent of the given node.
            assert(...
                ID >= 1 && ID <= numel(obj.parent) ...
                , 'No node with ID %d.' ...
                , ID ...
            );
            
            ID = obj.parent(ID);
        end
        
        function n = n_nodes(obj)
            %% N_NODES  Return the number of nodes in the Tree. 
            n = numel(obj.node);
        end

        function result = path_to_root(obj, ID)
            %% PATH_TO_ROOT  Path from node ID to the Tree root. 
            %
            
            result = ID;
            while result(end) ~= 1
                result(end+1)=obj.parent(result(end)); %#ok<AGROW>
            end
        end
    end
end

