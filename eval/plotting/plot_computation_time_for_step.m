function plot_computation_time_for_step(experiment_result, k, optional)
    %% PLOT_RUNTIME_FOR_STEP plots the timing of the specified timestep where t=0 is the start of the first hlc_init measurement
    %%      IMPORTANT: This function expects a result as input in which the controller_start_times were normalized before
    %%                 by normalize_timing_results

    arguments
        experiment_result (1, 1) ExperimentResult;
        k uint64;
        optional.fig (1, 1) matlab.ui.Figure = gcf;
    end

    hold_before = ishold;

    options = experiment_result.options;

    % Configure, which field names in the timing object are relevant, dependent on the used controller
    if ~(options.is_prioritized && options.computation_mode ~= ComputationMode.sequential)
        error('The graph is currently only supported for results of prioritized, distributed execution.');
    end

    field_names = [ ...
                       "measure", ...
                       "analyze_reachability", ...
                       "receive_from_others", ...
                       "couple", ...
                       "prioritize", ...
                       "group", ...
                       "optimize", ...
                   ];

    % Find time of HLC which starts loop last
    measure_timings = vertcat(experiment_result.timing.measure);
    measure_start_points = measure_timings(1:2:end, :);
    t0 = max(measure_start_points(:, k));

    for field_i = 1:length(field_names)
        field_name = field_names(field_i);
        time_to_draw = nan(2, options.amount);

        for veh_i = 1:options.amount

            timings = experiment_result.timing(veh_i);

            t_start = timings.(field_name)(1, k) - t0; % Normalize (see above)
            duration = timings.(field_name)(2, k);
            time_to_draw(:, veh_i) = [t_start, t_start + duration] * 10^3; % Scale to ms
        end

        plot_handle(:, field_i) = plot(time_to_draw, [1:options.amount; 1:options.amount], 'SeriesIndex', field_i, ...
            LineWidth = 5, Tag = 'box_as_line'); %#ok<AGROW>
        hold on;
    end

    legend(plot_handle(1, :), strrep(cellstr(field_names), '_', ' '), Location = 'best');
    xlabel('Time [ms]');
    xlim([-5, ceil(max(time_to_draw(2, :)))] + 5);
    ylabel('Vehicle ID');
    yticks(1:options.amount);
    ylim([1 - 0.2, options.amount + 0.2]);

    set_figure_properties(optional.fig, ExportFigConfig.document());

    if ~hold_before
        hold off;
    end

end
