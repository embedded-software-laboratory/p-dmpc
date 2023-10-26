function reference_path_struct = sample_reference_trajectory(n_samples, reference_path, vehicle_x, vehicle_y, step_size)
    % SAMPLE_REFERENCE_TRAJECTORY  Computes equidistant points along a piecewise linear curve. The first
    % point is the point on the curve closest to the given point
    % (vehicle_x,vehicle_y). All following points are on the curve with a
    % distance of 'step_size' to their predecessor.
    %
    % Arguments:
    %       n_samples: number of points created
    %       reference_path: piecewise linear curve [x1 y1; x2 y2; ...]
    %       vehicle_x,vehicle_y: start point
    %       step_size: Distance between points [d1; d2; ...]
    % Returns: ReferencePoints [x1 y1; x2 y2; ...] and corresponding
    % ReferenceIndex (point index)

    reference_path_struct = struct;
    reference_path_struct.ReferencePoints = zeros(n_samples, 2);
    reference_path_struct.ReferenceIndex = zeros(n_samples, 1);

    [~, ~, xp, yp, ~, ~, TrajectoryIndex] = get_arc_distance_to_endpoint( ...
        vehicle_x, ...
        vehicle_y, ...
        reference_path(:, 1), ...
        reference_path(:, 2) ...
    );

    nLinePieces = size(reference_path, 1);
    currentPoint = [xp yp];

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
        remainingLength = norm(currentPoint - reference_path(TrajectoryIndex, :), 2);

        if remainingLength > step_size(i) || TrajectoryIndex == nLinePieces

            % lanelets have overlapping points, which cannot be used to normalize
            while (reference_path(TrajectoryIndex, 1) == reference_path(TrajectoryIndexLast, 1) && reference_path(TrajectoryIndex, 2) == reference_path(TrajectoryIndexLast, 2) && TrajectoryIndexLast > 1)
                TrajectoryIndexLast = TrajectoryIndexLast - 1;
            end

            %normalize is used to construct a unit vector wrt the line segment
            %if the remaining lenght is larger than step size, then make a
            %step further first until the remaining length is less than the
            %step size
            currentPoint = currentPoint + step_size(i) * normalize(reference_path(TrajectoryIndex, :) - reference_path(TrajectoryIndexLast, :));

        else

            while remainingLength < step_size(i)
                % while the remaining length is shorter than step_size,
                % update the currentPoint to the current refPoint and update the
                % current reference point to the next refPoint until the
                % first refPoint after one step_size is found

                reflength = remainingLength;
                currentPoint = reference_path(TrajectoryIndex, :);

                TrajectoryIndexLast = TrajectoryIndex;
                TrajectoryIndex = min(TrajectoryIndex + 1, nLinePieces);

                is_vehicle_at_end = TrajectoryIndex == nLinePieces;

                if (is_loop && is_vehicle_at_end)
                    TrajectoryIndex = 1;
                end

                remainingLength = remainingLength + norm(currentPoint - reference_path(TrajectoryIndex, :), 2);
            end

            currentPoint = currentPoint + (step_size(i) - reflength) * normalize(reference_path(TrajectoryIndex, :) - reference_path(TrajectoryIndexLast, :));
            %             disp(['trajectoryIndex is :',num2str(TrajectoryIndex)])
        end

        % record step
        reference_path_struct.ReferencePoints(i, :) = currentPoint;
        reference_path_struct.ReferenceIndex(i, :) = TrajectoryIndex;

    end

end

function y = normalize(x)
    y = x / norm(x, 2);
end
