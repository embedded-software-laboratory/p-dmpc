function yaw = calculate_yaw(reference_path)
    % calculate_yaw - Calculates the yaw angle of the reference path
    arguments
        % path with
        % first column x-coordinates
        % second column y-coordinates
        reference_path (:, 2) double
    end

    yaw = zeros(length(reference_path), 1);

    % calculate intermediate yaws by using the previous and the next reference point
    dpos = reference_path(3:end, :) - reference_path(1:end - 2, :);
    yaw(2:end - 1) = atan2(dpos(:, 2), dpos(:, 1));

    % calculate first yaw by using the current and the next reference point
    yaw(1) = atan2( ...
        reference_path(2, 2) - reference_path(1, 2), ...
        reference_path(2, 1) - reference_path(1, 1) ...
    );

    % calculate last yaw by using the previous and the current reference point
    yaw(end) = atan2( ...
        reference_path(end, 2) - reference_path(end - 1, 2), ...
        reference_path(end, 1) - reference_path(end - 1, 1) ...
    );

end
