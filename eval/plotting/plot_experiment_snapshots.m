function plot_experiment_snapshots(experiment_result, step_indices, optional)
    % OVERVIEWPLOT  Export plot with multiple snapshots.
    arguments
        experiment_result (1, 1) ExperimentResult;
        step_indices (1, :) double
        optional.do_export (1, 1) logical = true;
        optional.fig (1, 1) matlab.ui.Figure = figure("Visible", "on");
    end

    experiment_result.options.options_plot_online.plot_priority = 0;
    plotter = PlotterOffline(experiment_result);
    plotter.set_figure_visibility(false);

    nFigs = numel(step_indices);
    n_figure_cols = 2;
    n_figure_rows = ceil(nFigs / n_figure_cols);

    set(0, 'CurrentFigure', optional.fig)

    tiledlayout(optional.fig, n_figure_rows, n_figure_cols);

    % show predictions for multiple timesteps
    for step = 1:nFigs
        step_idx = step_indices(step);

        plotter.set_time_step(step_idx);

        nexttile(step)
        hold on
        box on

        plotter.plot();

        if step > numel(step_indices) - n_figure_cols
            xlabel('$x$ [m]', Interpreter = 'LaTex')
        end

        if mod(step - 1, n_figure_cols) == 0
            ylabel('$y$ [m]', Interpreter = 'LaTex')
        end

        title(['Step ' num2str(step_idx)], Interpreter = 'LaTex');

        daspect([1 1 1]);
    end

    optional.fig.Children.TileSpacing = 'compact';
    set_figure_properties(optional.fig, ExportFigConfig.paper('paperheight', n_figure_rows * 4));

    if optional.do_export
        step_indices_str = sprintf("_%02d", step_indices);
        file_path = FileNameConstructor.path_to_accompanying_file( ...
            experiment_result, ...
            strcat("snapshots", step_indices_str, ".pdf") ...
        );
        export_fig(optional.fig, file_path);
    end

    if (~optional.fig.Visible); close(optional.fig); end

end
