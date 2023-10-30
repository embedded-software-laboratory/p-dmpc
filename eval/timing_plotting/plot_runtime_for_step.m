function plot_runtime_for_step(result, k)
    %% PLOT_RUNTIME_FOR_STEP plots the timing of the specified timestep

    arguments
        result struct;
        k uint64;
    end

    % Configure, which field names in the timing object are relevant, dependent on the used controller
    if result.scenario.options.is_prioritized & result.scenario.options.compute_in_parallel
        field_names = [ ...
                           "optimizer", ...
                           "fallback", ...
                           "publish_predictions" ...
                       ]; %"plan_single_vehicle", ...
        timings = result.timings_per_vehicle; % Will be unnecessary as soon as only one timing object exists
    else
        error('The graph is currently only supported for results of prioritized, distributed execution.');
    end

    colors = ["#0072BD", "#D95319", "#EDB120", "#7E2F8E", "#77AC30", "#4DBEEE", "#A2142F"];

    veh_i_to_draw = [1:result.scenario.options.amount; 1:result.scenario.options.amount];

    for field_i = 1:length(field_names)
        field_name = field_names(field_i);

        for veh_i = 1:result.scenario.options.amount
            t_start = timings.(field_name)(1, 1, k);
            duration = timings.(field_name)(2, 1, k);
            time_to_draw(:, veh_i) = [t_start, t_start + duration];
        end

        plt(:, field_i) = plot(time_to_draw, veh_i_to_draw, 'Color', colors(field_i), 'LineWidth', 5);
        hold on;
    end

    legend(plt(1, :), cellstr(field_names), 'Interpreter', 'none');
    xlabel('Time [ms]');
    ylabel('Vehicle');
    yticks(1:result.scenario.options.amount);
    ylim([1 - 0.2, result.scenario.options.amount + 0.2])

end
