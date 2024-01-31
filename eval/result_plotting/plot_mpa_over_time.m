function plot_mpa_over_time(experiment_result, optional)

    arguments
        experiment_result (1, 1) ExperimentResult;
        optional.do_export (1, 1) logical = true;
        optional.fig (1, 1) matlab.ui.Figure = figure("Visible", "on");
        optional.export_fig_cfg (1, 1) ExportFigConfig = ExportFigConfig.paper();
    end

    mpa = experiment_result.mpa;
    mpa.plot_over_time(fig = optional.fig);

    set_figure_properties(optional.fig, optional.export_fig_cfg);

    %export figure
    if optional.do_export
        file_path = FileNameConstructor.path_to_accompanying_file( ...
            experiment_result, ...
            'mpa_over_time.pdf' ...
        );
        export_fig(optional.fig, file_path);
    end

    if (~optional.fig.Visible); close(optional.fig); end

end
