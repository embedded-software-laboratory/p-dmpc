function experiment_result = normalize_timing_results(experiment_result)
    %NORMALIZE_TIMING_RESULTS normalizes the timing results by aligning
    % the synchronizing end of `receive_from_others` measurement

    arguments
        experiment_result (1, 1) ExperimentResult;
    end

    if experiment_result.options.computation_mode == ComputationMode.sequential
        return;
    end

    % Having received all messages from all vehicles is an event that occurs
    % almost simultaneously for all vehicles. Align by this time
    t_end_receive_from_others = zeros( ...
        experiment_result.options.amount, ...
        experiment_result.n_steps ...
    );

    for veh_i = 1:experiment_result.options.amount
        t_end_receive_from_others(veh_i, :) = sum(experiment_result.timing(veh_i).receive_from_others, 1);
    end

    t_end_receive_from_others_mean = mean(t_end_receive_from_others, 1);
    min_start_time_nanos = min([experiment_result.timing.controller_start_time]);

    % normalize according to min_start_time
    for veh_i = 1:experiment_result.options.amount

        % Align times with least squares
        b = t_end_receive_from_others(veh_i, :)' - t_end_receive_from_others_mean';
        A = ones(size(b, 1), 1);
        offset = lsqr(A, b);

        field_names = fieldnames(experiment_result.timing(veh_i));

        for field = field_names'

            if strcmp(field{1}, 'controller_start_time')
                experiment_result.timing(veh_i).(field{1})(1, :) = ...
                    min_start_time_nanos;
            else

                experiment_result.timing(veh_i).(field{1})(1, :) = ...
                    experiment_result.timing(veh_i).(field{1})(1, :) - offset;
            end

        end

    end

end
