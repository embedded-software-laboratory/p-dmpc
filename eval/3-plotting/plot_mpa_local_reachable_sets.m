function plot_mpa_local_reachable_sets(experiment_result, optional)

    arguments
        experiment_result (1, 1) ExperimentResult;
        optional.do_export (1, 1) logical = true;
        optional.fig (1, 1) matlab.ui.Figure = figure(Visible = "on");
        optional.export_fig_config (1, 1) ExportFigConfig = ExportFigConfig.paper();
    end

    mpa = MotionPrimitiveAutomaton(experiment_result.options);
    mpa.plot_local_reachable_sets(fig = optional.fig);

    set_figure_properties(optional.fig, optional.export_fig_config);

    %export figure
    if optional.do_export
        file_path = FileNameConstructor.path_to_accompanying_file( ...
            experiment_result, ...
            'mpa_reachable_sets.pdf' ...
        );
        export_fig(optional.fig, file_path);
    end

    if (~optional.fig.Visible); close(optional.fig); end

end
