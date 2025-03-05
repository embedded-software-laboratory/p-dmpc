function plot_experiment_snapshots(experiment_result, step_indices, optional)
    % PLOT_EXPERIMENT_SNAPSHOTS  Export plot with multiple snapshots.
    arguments
        experiment_result (1, 1) ExperimentResult;
        step_indices (1, :) double
        optional.do_export (1, 1) logical = true;
        optional.base_folder (1, 1) string = FileNameConstructor.experiment_result_folder_path( ...
                experiment_result.options ...
            );
        optional.fig (1, 1) matlab.ui.Figure = figure(visible = 'off');
        optional.n_figure_cols (1, 1) double = 2;
    end

    experiment_result.options.options_plot_online.plot_priority = 0;
    plotter = PlotterOffline(experiment_result);
    plotter.set_figure_visibility(false);
    plotter.set_export_fig_config(ExportFigConfig.paper);
    scenario = Scenario.create(experiment_result.options);

    nFigs = numel(step_indices);
    n_figure_cols = optional.n_figure_cols;
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

        if ~isempty(scenario.road_raw_data) && ~isempty(scenario.road_raw_data.lanelet)
            plot_lanelets(scenario.road_raw_data.lanelet)
        end
        plotter.plot();

        if step > numel(step_indices) - n_figure_cols
            xlabel('$x$ [m]', Interpreter = 'LaTex')
        end

        if mod(step - 1, n_figure_cols) == 0
            ylabel('$y$ [m]', Interpreter = 'LaTex')
        end

        title(['Step ' num2str(step_idx)], Interpreter = 'LaTex');

        xlim(scenario.plot_limits(1, :));
        ylim(scenario.plot_limits(2, :));
        daspect([1 1 1])
    end

    optional.fig.Children.TileSpacing = 'compact';
    set_figure_properties( ...
        optional.fig, ...
        ExportFigConfig.paper(paperheight = n_figure_rows * 15, paperwidth = n_figure_cols * 17) ...
    );

    if optional.do_export
        step_indices_str = sprintf("_%02d", step_indices);
        filename = strcat(experiment_result.file_name, "_snapshots", step_indices_str, ".png");
        file_path = fullfile( ...
            optional.base_folder, ...
            filename ...
        );
        export_fig(optional.fig, file_path, is_vector_graphic = false);
    end

    if (~optional.fig.Visible); close(optional.fig); end

end
