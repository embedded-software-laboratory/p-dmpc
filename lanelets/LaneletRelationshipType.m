classdef LaneletRelationshipType
% LANELETRELATIONSHIPTYPE  Different types of relationship of adjacent lanelets.
    
    properties (Constant)
        type_1 = 'successive'; % successive lanelets
        type_2 = 'adjacent left/right (same)'; % adjacent left or right lanelets in the same direction
        type_3 = 'merging'; % merging lanelets
        type_4 = 'forking'; % forking lanelets
        type_5 = 'intersecting'; % intersecting lanelets
        type_6 = 'adjacent left/right (opposite)'; % parallel lanelets in the opposite direction in the opposite direction
    end
end