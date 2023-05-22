classdef LaneletRelationshipType
    % LANELETRELATIONSHIPTYPE  Different types of relationship of adjacent lanelets.

    enumeration
        longitudinal % longitudinal-adjacent 1
        side % left- or right-adjacent 2
        merging % merging-adjacent 3
        forking % forking-adjacent 4
        crossing % crossing-adjacent 5
        same % the same lanelet 6
    end

end
