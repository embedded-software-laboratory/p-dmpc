function plot_runtime_multiple_experiments(results)
    %PLOT_RUNTIME_MULTIPLE_EXPERIMENTS draws median and max in a stacked bar plot for multiple experiments with different numbers of vehicles
    arguments
        % cell array with the result structs of all vehicles (columns= different results of vehicles, rows= different experiments)
        % the first row needs to contain one vehicle, the second two vehicles, ...
        results (:, :) cell;
    end

    n_experiments = size(results, 1);
    n_vehicles_max = n_experiments; % assume we have in the first experiment one vehicle and in the last n_experiments vehicles

    time_med = nan(1, n_experiments);
    time_max = nan(1, n_experiments);

    for i_experiment = 1:n_experiments
        n_steps = results{1, 1}.nSteps;
        times_per_vehicle = nan(n_steps - 1, i_experiment); % assume triangular matrix

        for i_vehicle = 1:i_experiment % assume triangular matrix
            times_per_vehicle(:, i_vehicle) = results{i_experiment, i_vehicle}.timings_general.hlc_step(2, :, 2:n_steps);
        end

        time_med(i_experiment) = median(times_per_vehicle, "all");
        time_max(i_experiment) = max(times_per_vehicle, [], "all");

    end

    times_to_plot(:, 1) = time_med;
    times_to_plot(:, 2) = time_max - time_med;

    figHandle = figure();
    bar(times_to_plot, 'stacked');
    xlabel('Number of vehicles in experiment');
    ylabel('Time [ms]');
    legend(["Median", "Max"], 'Interpreter', 'none');

    set_figure_properties(figHandle, ExportFigConfig.document());

end
