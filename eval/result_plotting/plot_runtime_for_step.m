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
    if options.is_prioritized & options.compute_in_parallel
        field_names = [ ...
                           "hlc_step", ...
                           "traffic_situation_update", ...
                           "optimizer", ...
                           "fallback", ...
                           "publish_predictions" ...
                       ]; %"plan_single_vehicle", ...
    else
        error('The graph is currently only supported for results of prioritized, distributed execution.');
    end

    % Find minimum in start of first measurement, i.e., "hlc_init", to normalize time by that.
    t0 = realmax;

    for veh_i = 1:options.amount
        t0 = min(t0, results{veh_i}.timings_general.hlc_step(1, 1, k));
    end

    figHandle = figure();

    for field_i = 1:length(field_names)
        field_name = field_names(field_i);

        for veh_i = 1:options.amount

            if isfield(results{1, veh_i}.timings_per_vehicle(veh_i), field_name)
                timings = results{1, veh_i}.timings_per_vehicle(veh_i);
            else
                timings = results{1, veh_i}.timings_general(1);
            end

            t_start = timings.(field_name)(1, 1, k) - t0; % Normalize (see above)
            duration = timings.(field_name)(2, 1, k);
            time_to_draw(:, veh_i) = [t_start, t_start + duration] * 10^3; % Scale to ms
        end

        plt(:, field_i) = plot(time_to_draw, [1:options.amount; 1:options.amount], 'SeriesIndex', field_i, ...
            'LineWidth', 5, 'Tag', 'box_as_line');
        hold on;
    end

    legend(plt(1, :), strrep(cellstr(field_names), '_', ' '));
    xlabel('Time [ms]');
    ylabel('Vehicle');
    yticks(1:options.amount);
    ylim([1 - 0.2, options.amount + 0.2]);

    set_figure_properties(figHandle, ExportFigConfig.document());

    % Export
    if optional.do_export
        folder_path = FileNameConstructor.gen_results_folder_path( ...
            results{1, 1}.options ...
        );
        filename = strcat('runtime_step_', string(k), '.pdf');
        export_fig(figHandle, fullfile(folder_path, filename));
        close(figHandle);
    end

end
