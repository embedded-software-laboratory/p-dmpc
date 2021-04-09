% for hierarchical collision check
% offset has to be at least the maximum length from CG to a point on the edge of each vehicle model
function iscandidate = collision_candidate(midpoint1,midpoint2,displacement1,displacement2,offset1,offset2)
    
    % ref distance is the sum of the circles radii aproximating the maneuver area
    ref_dist = (displacement1+displacement2)/2 + offset1 + offset2;
    dist = euclidean_distance(midpoint1(1),midpoint1(2),midpoint2(1),midpoint2(2));
    
    if dist < ref_dist
        iscandidate = true;
        return;
    end
    
    iscandidate = false;

end