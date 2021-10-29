function [signed_distance_min, arclength_min, x_min, y_min, index_min ] = getShortestDistance(curve_x,curve_y,x,y)
% GETSHORTESTDISTANCE   Finds the point on a piecewise linear curve that is closest to a
%                       given point.
% Params:
%     curve_x, curve_y:
%         A polygonal chain (a.k.a. piecewise linear curve)
%     x,y:
%         The point to be projected onto the curve
% Returns:
%     x_min, y_min:
%         The projected point on the curve.
%     arclength_min:
%         Arc length on the curve between (curve_x(1),curve_y(1)) and 
%         (x_min,y_min).
%     signed_distance_min:
%         Signed distance between (x_min,y_min) and (x,y).
%         Left ~ positive, right ~ negative.

    assert(length(x)==1);
    assert(length(y)==1);
    assert(length(curve_x) == length(curve_y));
    assert(length(curve_x) >= 2);
    arclength_sum = 0;

    
    % Guess first point as minimum
    x_min = curve_x(1);
    y_min = curve_y(1);
    arclength_min=0;
    signed_distance_min = sqrt((x-curve_x(1))^2 +(y-curve_y(1))^2);
    index_min = 2;
    
    for j = 2:(length(curve_x))
        [xp, yp, signed_distance, lambda, piecelength] =  Projection2D(curve_x(j-1),curve_y(j-1),curve_x(j),curve_y(j),x,y);

        % Projected point is between the two ref points curve_x(j-1) and curve_x(j).
        if (0 < lambda) && (lambda < 1)
            if abs(signed_distance) < abs(signed_distance_min)
                x_min = xp; %The projected point on the curve.
                y_min = yp;
                signed_distance_min = signed_distance;
                arclength_min= arclength_sum + lambda * piecelength;
                index_min = j;
            end
            
        % Projected point is beyond the two ref points curve_x(j-1) and curve_x(j).    
        else
            d_end = sqrt((x-curve_x(j))^2 +(y-curve_y(j))^2);
            if  abs(d_end) < abs(signed_distance_min)
                x_min = xp; %The projected point on the curve.
                y_min = yp;
                signed_distance_min = signum(signed_distance)*d_end;
                arclength_min= arclength_sum + piecelength;
                index_min = j;
            end
        end
        arclength_sum = arclength_sum+piecelength;
    end
    
end

function sign = signum(x)
    if x < 0
        sign = -1;
        return;
    end
        sign = 1;
end