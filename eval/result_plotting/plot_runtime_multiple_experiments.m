function plot_runtime_multiple_experiments(experiment_results, optional)
    %PLOT_RUNTIME_MULTIPLE_EXPERIMENTS draws median and max in a stacked
    % bar plot for multiple experiments with different numbers of vehicles
    arguments
        % cell array with the result structs of all vehicles
        % (columns= different results of vehicles, rows= different experiments)
        % the first row needs to contain one vehicle, the second two vehicles, ...
        experiment_results (:, :) cell;
        optional.do_export (1, 1) logical = true;
    end

    % assume we have in the first experiment one vehicle and in the last n_experiments vehicles
    n_experiments = size(experiment_results, 1);

    time_med = nan(1, n_experiments);
    time_max = nan(1, n_experiments);

    for i_experiment = 1:n_experiments
        n_steps = experiment_results{1, 1}.n_steps;
        times_per_vehicle = nan(n_steps - 1, i_experiment); % assume triangular matrix

        for i_vehicle = 1:i_experiment % assume triangular matrix
            times_per_vehicle(:, i_vehicle) = experiment_results{i_experiment, i_vehicle}.timings_general.hlc_step(2, :, 2:n_steps);
        end

        time_med(i_experiment) = median(times_per_vehicle, "all");
        time_max(i_experiment) = max(times_per_vehicle, [], "all");

    end

    times_to_plot(:, 1) = time_med * 10^3;
    times_to_plot(:, 2) = (time_max - time_med) * 10^3;

    figure_handle = figure();
    bar(times_to_plot, 'stacked');
    xlabel('Number of vehicles in experiment');
    ylabel('Time [ms]');
    legend(["Median", "Max"], 'Interpreter', 'none');

    set_figure_properties(figure_handle, ExportFigConfig.document());

    % Export
    if optional.do_export
        folder_path = FileNameConstructor.gen_results_folder_path( ...
            experiment_results{1, 1}.options ...
        );
        filename = 'runtime_multiple_experiments.pdf';
        export_fig(figure_handle, fullfile(folder_path, filename));
        close(figure_handle);
    end

end
