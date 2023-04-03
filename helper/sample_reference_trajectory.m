function reference = sample_reference_trajectory(n_samples, reference_trajectory, vehicle_x, vehicle_y, step_size, auto_updated_path, is_prioritized)
    % SAMPLEREFERENCETRAJETORY  Computes equidistant points along a piecewise linear curve. The first
    % point is the point on the curve closest to the given point
    % (vehicle_x,vehicle_y). All following points are on the curve with a
    % distance of 'step_size' to their predecessor.
    %
    % Arguments:
    %       n_samples: number of points created
    %       reference_trajectory: piecewise linear curve [x1 y1; x2 y2; ...]
    %       vehicle_x,vehicle_y: start point
    %       step_size: Distance between points [d1; d2; ...]
    % Returns: ReferencePoints [x1 y1; x2 y2; ...] and corresponding
    % ReferenceIndex (point index)

    reference = struct;
    reference.ReferencePoints = zeros(n_samples, 2);
    reference.ReferenceIndex = zeros(n_samples, 1);

    % change another computationally fast way to calculate the closest
    % point, the results may differ marginally but not influence the
    % performance.
    %     [~, ~, xp, yp, TrajectoryIndex ] = getShortestDistance(reference_trajectory(:,1),reference_trajectory(:,2),vehicle_x,vehicle_y);
    [~, ~, xp, yp, ~, ~, TrajectoryIndex] = get_arc_distance_to_endpoint(vehicle_x, vehicle_y, reference_trajectory(:, 1), reference_trajectory(:, 2));
    current_trajectory_index = TrajectoryIndex; % store the trajectory index of the current position of the vehicle

    nLinePieces = size(reference_trajectory, 1);
    currentPoint = [xp yp];

    % If the first and the last Point of the reference path are the same(here within a small enough distance),
    % we think the reference is in a loop.
    Is_refPath_loop = norm((reference_trajectory(1, :) - reference_trajectory(end, :)), 2) < 1e-8;
    Is_refPoint_last = TrajectoryIndex == nLinePieces;
    TrajectoryIndexLast = TrajectoryIndex - 1;

    % If the refPath is in a loop and the refPoint is the last one, change
    % the TrajectoryIndex to 1
    if Is_refPath_loop && Is_refPoint_last
        TrajectoryIndex = 1;
    end

    for i = 1:n_samples
        remainingLength = norm(currentPoint - reference_trajectory(TrajectoryIndex, :), 2);

        if remainingLength > step_size(i) || TrajectoryIndex == nLinePieces

            % lanelets have overlapping points, which cannot be used to normalize
            while (reference_trajectory(TrajectoryIndex, 1) == reference_trajectory(TrajectoryIndexLast, 1) && reference_trajectory(TrajectoryIndex, 2) == reference_trajectory(TrajectoryIndexLast, 2) && TrajectoryIndexLast > 1)
                TrajectoryIndexLast = TrajectoryIndexLast - 1;
            end

            %normalize is used to construct a unit vector wrt the line segment
            %if the remaining lenght is larger than step size, then make a
            %step further first until the remaining length is less than the
            %step size
            currentPoint = currentPoint + step_size(i) * normalize(reference_trajectory(TrajectoryIndex, :) - reference_trajectory(TrajectoryIndexLast, :));

        else

            while remainingLength < step_size(i)
                % while the remaining length is shorter than step_size,
                % update the currentPoint to the current refPoint and update the
                % current reference point to the next refPoint until the
                % first refPoint after one step_size is found

                reflength = remainingLength;
                currentPoint = reference_trajectory(TrajectoryIndex, :);

                TrajectoryIndexLast = TrajectoryIndex;
                TrajectoryIndex = min(TrajectoryIndex + 1, nLinePieces);

                Is_refPoint_last = TrajectoryIndex == nLinePieces;

                if (Is_refPath_loop && Is_refPoint_last)
                    TrajectoryIndex = 1;
                elseif (Is_refPoint_last && auto_updated_path)
                    % to prevent endless loop if vehicle is closer to last lane after an automated update than to the first lane
                    TrajectoryIndex = 1;
                end

                remainingLength = remainingLength + norm(currentPoint - reference_trajectory(TrajectoryIndex, :), 2);
            end

            currentPoint = currentPoint + (step_size(i) - reflength) * normalize(reference_trajectory(TrajectoryIndex, :) - reference_trajectory(TrajectoryIndexLast, :));
            %             disp(['trajectoryIndex is :',num2str(TrajectoryIndex)])
        end

        % record step
        reference.ReferencePoints(i, :) = currentPoint;

        if auto_updated_path && is_prioritized
            % if path has been updated automatically, set index manually to first lane
            reference.ReferenceIndex(i, :) = 2;
        else
            reference.ReferenceIndex(i, :) = TrajectoryIndex;
        end

    end

end

function y = normalize(x)
    y = x / norm(x, 2);
end
