function reference_path_struct = sample_reference_path(n_samples, reference_path, x_current, y_current, step_distances)
    % SAMPLE_REFERENCE_TRAJECTORY  Computes equidistant points along a piecewise linear curve. The first
    % point is the point on the curve closest to the given point
    % (x_current,y_current). All following points are on the curve with a
    % distance of 'step_distances' to their predecessor.
    %
    % Arguments:
    %       n_samples: number of points created
    %       reference_path: piecewise linear curve [x1 y1; x2 y2; ...]
    %       x_current,y_current: start point
    %       step_distances: Distance between points [d1; d2; ...]
    % Returns: path [x1 y1; x2 y2; ...] and corresponding
    % points_index (point index)

    reference_path_struct = struct;
    reference_path_struct.path = zeros(n_samples, 2);
    reference_path_struct.points_index = zeros(n_samples, 1);

    [~, ~, xp, yp, ~, ~, TrajectoryIndex] = get_arc_distance_to_endpoint( ...
        x_current, ...
        y_current, ...
        reference_path(:, 1), ...
        reference_path(:, 2) ...
    );

    nLinePieces = size(reference_path, 1);
    current_reference_point = [xp yp];

    % If the first and the last Point of the reference path are the same(here within a small enough distance),
    % we think the reference is in a loop.
    is_loop = norm((reference_path(1, :) - reference_path(end, :)), 2) < 1e-8;
    is_vehicle_at_end = TrajectoryIndex == nLinePieces;
    TrajectoryIndexLast = TrajectoryIndex - 1;

    % If the reference_path is in a loop and the refPoint is the last one, change
    % the TrajectoryIndex to 1
    if is_loop && is_vehicle_at_end
        TrajectoryIndex = 1;
    end

    for i = 1:n_samples
        remainingLength = norm(current_reference_point - reference_path(TrajectoryIndex, :), 2);

        if remainingLength > step_distances(i) || TrajectoryIndex == nLinePieces

            % lanelets have overlapping points, which cannot be used to normalize
            while (reference_path(TrajectoryIndex, 1) == reference_path(TrajectoryIndexLast, 1) && reference_path(TrajectoryIndex, 2) == reference_path(TrajectoryIndexLast, 2) && TrajectoryIndexLast > 1)
                TrajectoryIndexLast = TrajectoryIndexLast - 1;
            end

            %normalize is used to construct a unit vector wrt the line segment
            %if the remaining lenght is larger than step size, then make a
            %step further first until the remaining length is less than the
            %step size
            current_reference_point = current_reference_point + step_distances(i) * normalize(reference_path(TrajectoryIndex, :) - reference_path(TrajectoryIndexLast, :));

        else

            while remainingLength < step_distances(i)
                % while the remaining length is shorter than step_distances,
                % update the current_reference_point to the current refPoint and update the
                % current reference point to the next refPoint until the
                % first refPoint after one step_distances is found

                reflength = remainingLength;
                current_reference_point = reference_path(TrajectoryIndex, :);

                TrajectoryIndexLast = TrajectoryIndex;
                TrajectoryIndex = min(TrajectoryIndex + 1, nLinePieces);

                is_vehicle_at_end = TrajectoryIndex == nLinePieces;

                if (is_loop && is_vehicle_at_end)
                    TrajectoryIndex = 1;
                end

                remainingLength = remainingLength + norm(current_reference_point - reference_path(TrajectoryIndex, :), 2);
            end

            current_reference_point = current_reference_point + (step_distances(i) - reflength) * normalize(reference_path(TrajectoryIndex, :) - reference_path(TrajectoryIndexLast, :));
            %             disp(['trajectoryIndex is :',num2str(TrajectoryIndex)])
        end

        % record step
        reference_path_struct.path(i, :) = current_reference_point;
        reference_path_struct.points_index(i, :) = TrajectoryIndex;

    end

end

function y = normalize(x)
    y = x / norm(x, 2);
end
