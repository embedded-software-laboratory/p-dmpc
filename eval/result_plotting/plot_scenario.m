function plot_scenario(experiment_result, optional)

    arguments
        experiment_result (1, 1) ExperimentResult;
        optional.do_export (1, 1) logical = true;
        optional.fig (1, 1) matlab.ui.Figure = figure("Visible", "on");
        optional.export_fig_cfg (1, 1) ExportFigConfig = ExportFigConfig.paper();
    end

    scenario = create_scenario(experiment_result.options);
    scenario.plot(experiment_result.options, fig = optional.fig);

    set_figure_properties(optional.fig, ExportFigConfig.paper("paperheight", 6))

    %export figure
    if optional.do_export
        file_path = FileNameConstructor.path_to_accompanying_file( ...
            experiment_result, ...
            'scenario.pdf' ...
        );
        export_fig(optional.fig, file_path);
    end

    if (~optional.fig.Visible); close(optional.fig); end

end
