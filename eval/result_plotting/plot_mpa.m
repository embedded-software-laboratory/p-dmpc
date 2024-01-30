function plot_mpa(experiment_result, optional)

    arguments
        experiment_result (1, 1) ExperimentResult;
        optional.y_lim (1, 2) double = [-0.1, 1.0];
        optional.x_lim (1, 2) double = rad2deg(pi / 5 * [-1, 1]);
        optional.k (1, 1) double = 1;
        optional.with_labels (1, 1) logical = true;
        optional.do_export (1, 1) logical = true;
        optional.fig (1, 1) matlab.ui.Figure = figure("Visible", "on");
        optional.export_fig_cfg (1, 1) ExportFigConfig = ExportFigConfig.paper();
    end

    mpa = experiment_result.mpa;

    mpa.plot( ...
        y_lim = optional.y_lim, ...
        x_lim = optional.x_lim, ...
        k = optional.k, ...
        fig = optional.fig, ...
        with_labels = optional.with_labels ...
    );

    set_figure_properties(optional.fig, optional.export_fig_cfg);

    %export figure
    if optional.do_export
        file_path = FileNameConstructor.path_to_accompanying_file( ...
            experiment_result, ...
            'mpa.pdf' ...
        );
        export_fig(optional.fig, file_path);
    end

    if (~optional.fig.Visible); close(optional.fig); end

end
