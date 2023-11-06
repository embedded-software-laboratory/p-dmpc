function results = normalize_timing_results(results)
    %NORMALIZE_TIMING_RESULTS normalizes the timing results of one experiment by the controller_start_time of each timing object

    arguments
        results (1, :) cell; % cell array with the result structs of all vehicles
    end

    % find minimum controller start time
    min_start_time = realmax;

    for veh_i = 1:results{1, 1}.scenario.options.amount
        min_start_time = min(min_start_time, results{1, veh_i}.timings_per_vehicle(veh_i).controller_start_time);
        min_start_time = min(min_start_time, results{1, veh_i}.timings_general.controller_start_time);
    end

    % normalize according to min_start_time
    for veh_i = 1:results{1, 1}.scenario.options.amount
        results{1, veh_i}.timings_per_vehicle = normalize_one_timing_result(results{1, veh_i}.timings_per_vehicle, min_start_time, veh_i);
        results{1, veh_i}.timings_general = normalize_one_timing_result(results{1, veh_i}.timings_general, min_start_time, 1);
    end

end

function timings = normalize_one_timing_result(timings, min_start_time, veh_i)
    field_names = fieldnames(timings, '-full');

    % Store diff to minimal timing in controller_start_time
    difference_to_min_in_seconds = double(timings(veh_i).controller_start_time - min_start_time) * 10^-6;

    for field_i = 1:length(field_names)

        if strcmp(field_names{field_i}, 'controller_start_time')
            continue;
        end

        timings(veh_i).(field_names{field_i})(1, 1, :) = ...
            timings(veh_i).(field_names{field_i})(1, 1, :) + difference_to_min_in_seconds;
    end

end
