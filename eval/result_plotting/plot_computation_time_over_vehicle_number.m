function plot_computation_time_over_vehicle_number(experiment_results, optional)

    arguments
        % contains elements with varying number of vehicles
        experiment_results (1, :) ExperimentResult;
        optional.do_export (1, 1) logical = true;
        optional.fig (1, 1) matlab.ui.Figure = figure(visible = "on");
        optional.export_fig_cgf (1, 1) ExportFigConfig = ExportFigConfig.paper();
    end

    n_experiments = length(experiment_results);

    n_vehicles = zeros(1, n_experiments);

    time_med = nan(1, n_experiments);
    time_max = nan(1, n_experiments);

    for i_experiment = 1:n_experiments
        n_steps = experiment_results(i_experiment).n_steps;

        times_per_vehicle = nan(n_steps - 1, i_experiment);
        n_vehicles(i_experiment) = experiment_results(i_experiment).options.amount;

        for i_vehicle = 1:n_vehicles(i_experiment)
            times_per_vehicle(:, i_vehicle) = experiment_results(i_experiment).timing(i_vehicle).control_loop(2, :, 2:n_steps);
        end

        time_med(i_experiment) = median(times_per_vehicle, "all");
        time_max(i_experiment) = max(times_per_vehicle, [], "all");

        % plot with axis on both sides and logarithmic scale
        set(0, 'currentfigure', optional.fig);
        times_per_vehicle = reshape(times_per_vehicle, 1, []);
        boxchart(n_vehicles(i_experiment) * ones(size(times_per_vehicle)), times_per_vehicle * 1e3);
        hold on

    end

    set(gca, 'YScale', 'log');

    % set labels
    xlabel('Number of Vehicles', Interpreter = 'LaTex');
    ylabel('Computation Time [ms]', Interpreter = 'LaTex');

    set_figure_properties(optional.fig, ExportFigConfig.paper('paperheight', 12))

    if optional.do_export
        results_folder = FileNameConstructor.all_results();
        filepath = fullfile(results_folder, 'computation_time_boxplot.pdf');
        export_fig(optional.fig, filepath);
    end

    if (~optional.fig.Visible); close(optional.fig); end

    times_to_plot(:, 1) = time_med * 10^3;
    times_to_plot(:, 2) = (time_max - time_med) * 10^3;

    figure_handle = figure();
    bar(n_vehicles, times_to_plot, 'stacked');
    xlabel('Number of Vehicles');
    ylabel('Computation Time [ms]');
    legend(["Median", "Max"], Interpreter = 'none');

    set_figure_properties(figure_handle, ExportFigConfig.document());

    % Export
    if optional.do_export
        results_folder = FileNameConstructor.all_results();
        filename = 'computation_time_barplot.pdf';
        export_fig(figure_handle, fullfile(results_folder, filename));
    end

end
