function yaw = calculate_yaw(reference_path)
    % calculate_yaw - Calculates the yaw angle of the reference path
arguments
    % path with
    % first column x-coordinates
    % second column y-coordinates
    reference_path (:,2) double
end
    yaw = zeros(length(refPath), 1);

    % calculate intermediate yaws by using the previous and the next reference point 
    dpos = refPath(3:end, :) - refPath(1:end - 2, :);
    yaw(2:end - 1) = atan2(dpos(:, 2), dpos(:, 1));

    % calculate first yaw by using the current and the next reference point
    yaw(1) = atan2( ...
        refPath(2, 2) - refPath(1, 2), ...
        refPath(2, 1) - refPath(1, 1) ...
    );

    % calculate last yaw by using the previous and the current reference point
    yaw(end) = atan2( ...
        refPath(end, 2) - refPath(end - 1, 2), ...
        refPath(end, 1) - refPath(end - 1, 1) ...
    );

end
