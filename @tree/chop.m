function [obj, iterator] = chop(obj, node)
%% CHOP  Remove the target node and all subnodes from the given tree.
%
%
% e.g. 
% nodeIndex=34;
% myTree.chop(nodeIndex)

    iterator = obj.depthfirstiterator(node);
    iterator = sort(iterator,'descend');
    
    % Build new parent array
    np = obj.Parent;
    
    % Remove unwanted nodes
    np ( iterator ) = [];

    % Shift parent value: if a parent were after some nodes we removed, we
    % need to shift its value by an amount equal to the number of parent we
    % removed, and that were BEFORE the target parent
    for ii = iterator
        np(np > ii) = np(np > ii) - 1;
    end
    
    obj.Parent = np;
    obj.Node(iterator) = [];

end