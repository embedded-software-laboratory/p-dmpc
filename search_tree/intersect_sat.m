function collide = intersect_sat(shape1,shape2)
% INTERSECT_SAT         intersection test for two CONVEX polygon
%   COLLIDE = INTERSECT_SAT(shape1,shape2) returns true iff polygons defined by the vector shapei = [xs;ys] overlap
%   The implementation uses the separating axis theorem and therefore can only be used for convex polygons.
%   Behaviour for using the function for non-convex polygons is undefined

    collide = true;
    
    if ~intersect_a_b(shape1, shape2)
        collide = false;
    elseif ~intersect_a_b(shape2, shape1)
        collide = false;
    end
end

function collide = intersect_a_b(shape1,shape2)
    % all vectors between vertices, row 1: x row 2: y
    edge_vector = diff([shape1, shape1(:,1)],1,2);
    % projection axis orthogonal to edge vector
    axis = [-edge_vector(2,:);edge_vector(1,:)];
    % normalize axis
    normed_axis = axis./vecnorm(axis);
    % Project all vertices to all axes,
    % a row is the projection of all vertices to one axis
    dotprod1 = normed_axis'*shape1;
    % minimum of projected polygon to each axis
    minimum1 = min(dotprod1,[],2);
    maximum1 = max(dotprod1,[],2);
    dotprod2 = normed_axis'*shape2;
    minimum2 = min(dotprod2,[],2);
    maximum2 = max(dotprod2,[],2);
    d1 = minimum1 - maximum2;
    d2 = minimum2 - maximum1;
    if (any(d1>0)) || (any(d2>0))
        collide = false;
    else
        collide = true;
    end
end
