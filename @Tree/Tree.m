classdef Tree
%% TREE  A class implementing a Tree data structure.
%
% This class implements a simple Tree data structure. Each node can only
% have one parent, and store any kind of data. The root of the Tree is a
% privilieged node that has no parents and no siblings.
%
% Nodes are mainly accessed through their index. The index of a node is
% returned when it is added to the Tree, and actually corresponds to the
% order of addition.
%
% Basic methods to tarverse and manipulate trees are implemented. Most of
% them take advantage of the ability to create _coordinated_ trees: If
% a Tree is duplicated and only the new Tree data content is modified
% (i.e., no nodes are added or deleted), then the iteration order and the
% node indices will be the same for the two trees.
%
% Internally, the class simply manage an array referencing the node parent
% indices, and a cell array containing the node data. 

% Jean-Yves Tinevez <tinevez@pasteur.fr> March 2012
    
    properties (SetAccess = private)
        % Index of the parent node. The root of the Tree as a parent index
        % equal to 0.
        parent = [ 0 ]; %#ok<NBRAK>
    end
    
    properties (SetAccess = public)
        node = { [] }; % Hold the data at each node
        idx;
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
            obj.idx = obj.nodeCols();
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
        
        function [obj, ID] = addnode(obj, parent, data)
            %% ADDNODE attach a new node to a parent node
            % 
            % Tree = Tree.ADDNODE(parent_index, data) create a new node
            % with content 'data', and attach it as a child of the node
            % with index 'parent_index'. Return the modified Tree.
            % 
            % [ Tree, ID ] = Tree.ADDNODE(...) returns the modified Tree and
            % the index of the newly created node.
            
            if parent < 0 || parent > numel(obj.parent)
                error('MATLAB:Tree:addnode', ...
                    'Cannot add to unknown parent with index %d.\n', parent)
            end
            
            % Expand the cell by
            obj.node{ end + 1, 1 } = data;
            
            obj.parent = [
                obj.parent
                parent ];
            
            ID = numel(obj.node);
        
        end
        
        function [obj, IDs] = addnnodes(obj, parents, datas)
            %% ADDNODE attach multiple new nodes to a parent node
            % not a standard function
            
            np = numel(parents);
            
            if np == 1
                [obj, IDs] = obj.addnode(parents,datas{1});
            else
                IDs = (numel(obj.node)+1:numel(obj.node)+np);

                obj.node(IDs,1) = datas;

                obj.parent(IDs,1) = parents;
            end
        end
        
        function flag = is_leaf(obj, ID)
           %% IS_LEAF  Return true if given ID matches a leaf node.
           % A leaf node is a node that has no children.
           if ID < 1 || ID > numel(obj.parent)
                error('MATLAB:Tree:is_leaf', ...
                    'No node with ID %d.', ID)
           end
           
           parent = obj.parent;
           flag = ~any( parent == ID );
           
        end
        
        function IDs = find_leaves(obj,inBranch)
           %% FIND_LEAVES  Return the IDs of all the leaves of the Tree.
           %% if inBranch is a node index, then return only the leaves that include this node
           if nargin<2
                inBranch=[];
            end
           parents = obj.parent;
           IDs = (1 : numel(parents)); % All IDs
           IDs = setdiff(IDs, parents); % Remove those which are marked as parent

           if ~isempty(inBranch)
            for ii=length(IDs):-1:1
                thisPath=obj.pathtoroot(IDs(ii));
                f=find(thisPath==inBranch);
                if isempty(f)
                    IDs(ii)=[];
                end
            end
           end
           
        end
        
        function content = get(obj, ID)
            %% GET  Return the content of the given node ID.
            content = obj.node{ID};
        end

        function obj = set(obj, ID, content)
            %% SET  Set the content of given node ID and return the modifed Tree.
            obj.node{ID} = content;
        end

        
        function IDs = get_children(obj, ID)
        %% GET_CHILDREN  Return the list of ID of the children of the given node ID.
        % The list is returned as a line vector.
            parent = obj.parent;
            IDs = find( parent == ID );
            IDs = IDs';
        end
        
        function ID = get_parent(obj, ID)
        %% GET_PARENT  Return the ID of the parent of the given node.
            if ID < 1 || ID > numel(obj.parent)
                error('MATLAB:Tree:get_parent', ...
                    'No node with ID %d.', ID)
            end
            ID = obj.parent(ID);
        end
        
        function IDs = get_siblings(obj, ID)
            %% GET_SIBLINGS  Return the list of ID of the sliblings of the 
            % given node ID, including itself.
            % The list is returned as a column vector.
            if ID < 1 || ID > numel(obj.parent)
                error('MATLAB:Tree:get_siblings', ...
                    'No node with ID %d.', ID)
            end
            
            if ID == 1 % Special case: the root
                IDs = 1;
                return
            end
            
            parent = obj.parent(ID);
            IDs = obj.get_children(parent);
        end
        
        function n = n_nodes(obj)
            %% N_NODES  Return the number of nodes in the Tree. 
            n = numel(obj.nodes);
        end
        
        
    end
    
    % STATIC METHODS
    
    methods (Static)
        idx = nodeCols()
    end
    
end

