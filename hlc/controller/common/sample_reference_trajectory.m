function [reference_trajectory_struct, current_point_index] = sample_reference_trajectory(n_samples, reference_path, x_current, y_current, step_distances)
    % Computes points with given distances along a piecewise linear curve.
    % The point in current_point_index is the point on the curve closest
    % to the given point (x_current, y_current). All following points are
    % on the curve with a distance of 'step_distances(i)' to their predecessor
    % and are stored in reference_trajectory_struct.
    %
    % Output:
    %   reference_trajectory_struct struct with fields
    %       path (n_samples, 2) [x1 y1; x2 y2; ...]
    %       points_index (n_samples, 1) point indices
    %   current_point_index Index of point on the curve
    %       closest to given point (x_current, y_current)

    arguments
        n_samples (1, 1) double % number of points created
        reference_path (:, 2) double % piecewise linear curve [x1 y1; x2 y2; ...]
        x_current (1, 1) double % actual x coordinate
        y_current (1, 1) double % actual y coordinate
        step_distances (:, 1) double % distances between points [d1; d2; ...]
    end

    reference_trajectory_struct = struct;
    reference_trajectory_struct.path = zeros(n_samples, 2);
    reference_trajectory_struct.points_index = zeros(n_samples, 1);

    [~, ~, xp, yp, ~, ~, point_index] = get_arc_distance_to_endpoint( ...
        x_current, ...
        y_current, ...
        reference_path(:, 1), ...
        reference_path(:, 2) ...
    );
    current_point_index = point_index; % Keep point index corresponding to the projection of current_x,_y as return value

    n_line_pieces = size(reference_path, 1);
    current_reference_point = [xp yp];

    % If the first and the last Point of the reference path are the same(here within a small enough distance),
    % we think the reference is in a loop.
    is_loop = norm((reference_path(1, :) - reference_path(end, :)), 2) < 1e-8;
    is_vehicle_at_end = point_index == n_line_pieces;
    point_index_last = point_index - 1;

    % If the reference_path is in a loop and the refPoint is the last one, change
    % the point_index to 1
    if is_loop && is_vehicle_at_end
        point_index = 1;
    end

    for i = 1:n_samples
        remainingLength = norm(current_reference_point - reference_path(point_index, :), 2);

        if remainingLength > step_distances(i) || point_index == n_line_pieces

            % lanelets have overlapping points, which cannot be used to normalize
            while (reference_path(point_index, 1) == reference_path(point_index_last, 1) && reference_path(point_index, 2) == reference_path(point_index_last, 2) && point_index_last > 1)
                point_index_last = point_index_last - 1;
            end

            %normalize is used to construct a unit vector wrt the line segment
            %if the remaining lenght is larger than step size, then make a
            %step further first until the remaining length is less than the
            %step size
            current_reference_point = current_reference_point + step_distances(i) * normalize(reference_path(point_index, :) - reference_path(point_index_last, :));

        else

            while remainingLength < step_distances(i)
                % while the remaining length is shorter than step_distances,
                % update the current_reference_point to the current refPoint and update the
                % current reference point to the next refPoint until the
                % first refPoint after one step_distances is found

                reflength = remainingLength;
                current_reference_point = reference_path(point_index, :);

                point_index_last = point_index;
                point_index = min(point_index + 1, n_line_pieces);

                is_vehicle_at_end = point_index == n_line_pieces;

                if (is_loop && is_vehicle_at_end)
                    point_index = 1;
                end

                remainingLength = remainingLength + norm(current_reference_point - reference_path(point_index, :), 2);
            end

            current_reference_point = current_reference_point + (step_distances(i) - reflength) * normalize(reference_path(point_index, :) - reference_path(point_index_last, :));
            %             disp(['point_index is :',num2str(point_index)])
        end

        % record step
        reference_trajectory_struct.path(i, :) = current_reference_point;
        reference_trajectory_struct.points_index(i, :) = point_index;

    end

end

function y = normalize(x)
    y = x / norm(x, 2);
end
