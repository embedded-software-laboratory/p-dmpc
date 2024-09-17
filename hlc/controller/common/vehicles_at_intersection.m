function [ids, times] = vehicles_at_intersection(time_step, ids_old, times, threshold, states_measured, intersection_center, amount)

    if isempty(times)
        % this is empty only at the initial time step
        times = inf * ones(1, amount); % time step when vehicle enters the intersection
    end

    % vehicles are considered as at the intersection if their distances to
    % the intersection center point is smaller than a certain value
    distances_to_center = sqrt(sum((states_measured(:, 1:2) - intersection_center).^2, 2));
    ids = find(distances_to_center < threshold);

    ids_coming = setdiff(ids, ids_old);
    ids_leaving = setdiff(ids_old, ids);

    times(ids_coming) = time_step;
    times(ids_leaving) = inf; % set to inf if vehicle leaves the intersection
end
