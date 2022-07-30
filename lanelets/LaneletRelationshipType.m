classdef LaneletRelationshipType
% LANELETRELATIONSHIPTYPE  Different types of relationship of adjacent lanelets.
    
    properties (Constant)
        type_1 = 'logitudianl'; % longitudinal-adjacent
        type_2 = 'left/right';  % left- or right-adjacent
        type_3 = 'merging';     % merging-adjacent
        type_4 = 'forking';     % forking-adjacent
        type_5 = 'crossing';    % crossing-adjacent
        type_6 = 'same';        % the same lanelet
    end
end