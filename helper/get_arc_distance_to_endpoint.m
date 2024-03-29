function [arc_distance, arc_length, x_projected, y_projected, projection_distance, curve_new, idx_next] = get_arc_distance_to_endpoint(point_x, point_y, curve_x, curve_y)
    % GET_ARC_DISTANCE_TO_ENDPOINT This function calculate the distance from
    % the given point to the endpoint of the given curve. This is done by
    % firstly projecting the given point to the curve and calculating the arc
    % length from the projected point to curve's endpoint. The arc distance is
    % then sqrt(arc_length^2+projection_distance^2), where the
    % projection_distance is the distance between the given point and the
    % projected point on the curve.
    %
    % INPUT:
    %   point_x: x-position of the point to be projected
    %
    %   point_y: y-position of the point to be projected
    %
    %   curve_x: x-coordinates of the curve
    %
    %   curve_y: y-coordinates of the curve
    %
    % OUTPUT:
    %   arc_distance: sqrt(arc_length^2 +  projection_distance^2)
    %
    %   arc_length: arc length from the projected point on the curve to the
    %   endpoint of the curve, which is also the arc length of the new curve

    %   x_prjected: x-position of the projected point on the curve
    %
    %   y_prjected: y-position of the projected point on the curve
    %
    %   projection_distance: distance from the point to be projected and the
    %   prejected point on the curve
    %
    %   curve_new: a new curve with the prejected point as the starting
    %   point and all points after the prejected points on the original curve
    %
    %   idx_next: the index of the next point on the curve that is adjacent to
    %   the closest point on the curve to the target point
    %

    n_points = length(curve_x); % number of points in the curve

    curve = [curve_x, curve_y];

    squared_distances = sum([curve_x - point_x, curve_y - point_y].^2, 2); % ignore sqrt to save computation time
    [~, idx_closest] = min(squared_distances);

    % find which adjacent point in the curve is closer to the given point
    if idx_closest == 1
        idx_adjacent_point = 2;
        line_x_first = curve_x(idx_closest);
        line_y_first = curve_y(idx_closest);
        line_x_second = curve_x(idx_adjacent_point);
        line_y_second = curve_y(idx_adjacent_point);
        % delete the points before the second adjacent point since they are irrelevant to calculate the needed arc length
        curve_shortened = curve(idx_adjacent_point:end, :);
    elseif idx_closest == n_points
        idx_adjacent_point = n_points - 1;
        line_x_first = curve_x(idx_adjacent_point);
        line_y_first = curve_y(idx_adjacent_point);
        line_x_second = curve_x(idx_closest);
        line_y_second = curve_y(idx_closest);
        % delete the points before the second adjacent point since they are irrelevant to calculate the needed arc length
        curve_shortened = curve(idx_closest, :);
    else
        % find whether the left point or the right point of the cloestest point in the curve is closer to the given point
        [~, idx_adjacent_point_tmp] = min(squared_distances([idx_closest - 1, idx_closest + 1]));

        if idx_adjacent_point_tmp == 1
            % adjacent left point is colser to the given point than the adjacent right point
            idx_adjacent_point = idx_closest - 1;
            line_x_first = curve_x(idx_adjacent_point);
            line_y_first = curve_y(idx_adjacent_point);
            line_x_second = curve_x(idx_closest);
            line_y_second = curve_y(idx_closest);
            % delete the points before the second adjacent point since they are irrelevant to calculate the needed arc length
            curve_shortened = curve(idx_closest:end, :);
        elseif idx_adjacent_point_tmp == 2
            % adjacent right point is colser to the given point than the adjacent left point
            idx_adjacent_point = idx_closest + 1;
            line_x_first = curve_x(idx_closest);
            line_y_first = curve_y(idx_closest);
            line_x_second = curve_x(idx_adjacent_point);
            line_y_second = curve_y(idx_adjacent_point);
            % delete the points before the second adjacent point since they are irrelevant to calculate the needed arc length
            curve_shortened = curve(idx_adjacent_point:end, :);
        end

    end

    % project the given point to the line segment determined by the colsest point in the given curve and its adjacent point
    [x_projected, y_projected, projection_distance, lambda, ~] = projection_2d(line_x_first, line_y_first, line_x_second, line_y_second, point_x, point_y);

    point_projected = [x_projected, y_projected];

    % get the index of the next closest point
    idx_next = idx_closest;
    % case 1: if the projected point is on the left side of the middle
    % point of the line segment
    % case 2: if the projected point is on the right side of the line
    % segment
    % other cases: equal to the closest point
    if (lambda >= 0 && lambda <= 0.5) || lambda >= 1

        if idx_closest < n_points % not exceed the max index
            idx_next = idx_closest + 1;
        else
            idx_next = 1; % loop back
        end

    end

    % If the index of the next point on the curve that is adjacent to the
    % closest point on the curve to the target point smaller than two, we
    % change it to two to fullfill the requirement of another function.
    idx_next = max(2, idx_next);

    % add the projected point to the shortened curve
    if lambda < 1
        % the projected point is on the or on the left side of the second point of the line segment
        curve_new = [point_projected; curve_shortened];
    else % lambda>1
        % the projected point is on the right side of the second point of the line segment
        % delete the first point of the already shortened curve
        curve_new = [point_projected; curve_shortened(2:end, :)];
    end

    % calculate the arc length
    arc_length = sum(sqrt(sum(diff(curve_new, 1, 1).^2, 2)), 1);

    % calculate the arc distance
    arc_distance = sqrt(arc_length^2 + projection_distance^2);

end
