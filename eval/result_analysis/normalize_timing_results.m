function experiment_results = normalize_timing_results(experiment_results)
    %NORMALIZE_TIMING_RESULTS normalizes the timing results of one experiment by the controller_start_time of each timing object

    arguments
        experiment_results (1, :) cell; % cell array with the result structs of all vehicles
    end

    % find minimum controller start time
    min_start_time = realmax;

    for veh_i = 1:experiment_results{1, 1}.options.amount
        min_start_time = min(min_start_time, experiment_results{1, veh_i}.timing.controller_start_time);
    end

    % normalize according to min_start_time
    for veh_i = 1:experiment_results{1, 1}.options.amount
        experiment_results{1, veh_i}.timing = normalize_one_timing_result(experiment_results{1, veh_i}.timing, min_start_time, 1);
    end

end

function timings = normalize_one_timing_result(timings, min_start_time, veh_i)
    field_names = fieldnames(timings, '-full');

    % Store diff to minimal timing in controller_start_time
    difference_to_min_in_seconds = double(timings(veh_i).controller_start_time - min_start_time);

    for field_i = 1:length(field_names)

        if strcmp(field_names{field_i}, 'controller_start_time')
            timings(veh_i).controller_start_time = min_start_time;
        else
            timings(veh_i).(field_names{field_i})(1, 1, :) = ...
                timings(veh_i).(field_names{field_i})(1, 1, :) + difference_to_min_in_seconds;
        end

    end

end
