function plot_runtime_for_step(results, k)
    %% PLOT_RUNTIME_FOR_STEP plots the timing of the specified timestep

    arguments
        results (1, :) cell; % cell array with the result structs of all vehicles
        k uint64;
    end

    disp(results{1, 1});

    % useful variable for shorter notations
    options = results{1, 1}.scenario.options;

    % Configure, which field names in the timing object are relevant, dependent on the used controller
    if options.is_prioritized & options.compute_in_parallel
        field_names = [ ...
                           "optimizer", ...
                           "fallback", ...
                           "publish_predictions" ...
                       ]; %"plan_single_vehicle", ...
    else
        error('The graph is currently only supported for results of prioritized, distributed execution.');
    end

    % Configure the colors that shall be used (in this order)
    colors = ["#0072BD", "#D95319", "#EDB120", "#7E2F8E", "#77AC30", "#4DBEEE", "#A2142F"];

    for field_i = 1:length(field_names)
        field_name = field_names(field_i);

        for veh_i = 1:options.amount
            timings = results{1, veh_i}.timings_per_vehicle(veh_i);
            t_start = timings.(field_name)(1, 1, k);
            duration = timings.(field_name)(2, 1, k);
            time_to_draw(:, veh_i) = [t_start, t_start + duration];
        end

        plt(:, field_i) = plot(time_to_draw, [1:options.amount; 1:options.amount], 'Color', colors(field_i), 'LineWidth', 5);
        hold on;
    end

    legend(plt(1, :), cellstr(field_names), 'Interpreter', 'none');
    xlabel('Time [ms]');
    ylabel('Vehicle');
    yticks(1:options.amount);
    ylim([1 - 0.2, options.amount + 0.2])

end
