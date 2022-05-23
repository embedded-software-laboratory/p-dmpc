classdef CollisionType
% COLLISIONTYPE  Collision type.
    
    properties (Constant)
        type_1 = 'rear-end'; % rear-end collision, possible when two vehicles drive successively 
        type_2 = 'side-impact'; % side-impact collision, possible at merging lanelets or intersecting lanelets
        type_3 = 'head-on' ; % head-on collisions includes one vehicle colliding with the front end of another vehicle and vehicles crash with statical obstacles
    end
end