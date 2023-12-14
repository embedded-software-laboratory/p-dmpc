function plot_runtime_for_step(results, k, optional)
    %% PLOT_RUNTIME_FOR_STEP plots the timing of the specified timestep where t=0 is the start of the first hlc_init measurement
    %%      IMPORTANT: This function expects a result as input in which the controller_start_times were normalized before
    %%                 by normalize_timing_results

    arguments
        results (1, :) cell; % cell array with the result structs of all vehicles
        k uint64;
        optional.do_export (1, 1) logical = true;
    end

    % useful variable for shorter notations
    options = results{1, 1}.options;

    % Configure, which field names in the timing object are relevant, dependent on the used controller
    if ~(options.is_prioritized && options.computation_mode ~= ComputationMode.sequential)
        error('The graph is currently only supported for results of prioritized, distributed execution.');
    end

    field_names = [ ...
                       "measure", ...
                       "update_controlled_vehicles_traffic_info", ...
                       "receive_from_others", ...
                       "couple", ...
                       "prioritize", ...
                       "weigh", ...
                       "cut", ...
                       "optimize", ...
                   ];

    % Find minimum in start of first measurement, i.e., "hlc_init", to normalize time by that.
    t0 = realmax;

    for veh_i = 1:options.amount
        t0 = min(t0, results{veh_i}.timing.control_loop(1, 1, k));
    end

    figure_handle = figure();

    for field_i = 1:length(field_names)
        field_name = field_names(field_i);

        for veh_i = 1:options.amount

            timings = results{1, veh_i}.timing(1);

            t_start = timings.(field_name)(1, 1, k) - t0; % Normalize (see above)
            duration = timings.(field_name)(2, 1, k);
            time_to_draw(:, veh_i) = [t_start, t_start + duration] * 10^3; % Scale to ms
        end

        plot_handle(:, field_i) = plot(time_to_draw, [1:options.amount; 1:options.amount], 'SeriesIndex', field_i, ...
            'LineWidth', 5, 'Tag', 'box_as_line');
        hold on;
    end

    legend(plot_handle(1, :), strrep(cellstr(field_names), '_', ' '));
    xlabel('Time [ms]');
    ylabel('Vehicle');
    yticks(1:options.amount);
    ylim([1 - 0.2, options.amount + 0.2]);

    set_figure_properties(figure_handle, ExportFigConfig.document());

    % Export
    if optional.do_export
        folder_path = FileNameConstructor.gen_results_folder_path( ...
            results{1, 1}.options ...
        );
        filename = strcat('runtime_step_', string(k), '.pdf');
        export_fig(figure_handle, fullfile(folder_path, filename));
        close(figure_handle);
    end

end
