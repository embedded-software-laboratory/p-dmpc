classdef CollisionType
    % COLLISIONTYPES  Different types of potential collisions between two vehicles.

    enumeration
        from_rear % rear collision, possible when two vehicles drive successively
        from_side % side collision, possible at merging lanelets or intersecting lanelets
        from_front % head-on collisions includes one vehicle colliding with the front end of another vehicle and vehicles crash with statical obstacles
    end

end
