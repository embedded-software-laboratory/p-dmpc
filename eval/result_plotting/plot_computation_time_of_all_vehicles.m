function plot_computation_time_of_all_vehicles(experiment_results, optional)

    arguments
        experiment_results (:, :) ExperimentResult;
        optional.do_export (1, 1) logical = true;
        optional.fig (1, 1) matlab.ui.Figure = figure(visible = "on");
        optional.export_fig_cgf (1, 1) ExportFigConfig = ExportFigConfig.paper();
    end

    n_results = length(experiment_results);

    runtimes = zeros(experiment_results(1).n_steps, n_results);
    n_vehicles = zeros(1, n_results);

    for i = 1:n_results
        runtimes(:, i) = squeeze(experiment_results(i).timing.control_loop(2, 1, :));
        n_vehicles(i) = experiment_results(i).options.amount;
    end

    [~, order] = sort(n_vehicles);

    runtimes(:, :) = runtimes(:, order);
    n_Vehicles(:) = n_vehicles(order);

    % plot with axis on both sides and logarithmic scale
    set(0, 'currentfigure', optional.fig);
    boxplot(runtimes, n_Vehicles, 'MedianStyle', 'line');
    set(gca, 'YScale', 'log');

    % set labels
    xlabel('Number of Vehicles', 'Interpreter', 'LaTex');
    ylabel('Computation Time [s]', 'Interpreter', 'LaTex');

    set_figure_properties(optional.fig, ExportFigConfig.paper('paperheight', 12))

    if optional.do_export
        results_folder = FileNameConstructor.all_results();
        filepath = fullfile(results_folder, 'computation_time.pdf');
        export_fig(optional.fig, filepath);
    end

    if (~optional.fig.Visible); close(optional.fig); end

end
