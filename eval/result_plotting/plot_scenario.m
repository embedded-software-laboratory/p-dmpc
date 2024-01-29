function plot_scenario(options, optional)

    arguments
        options (1, 1) Config;
        optional.do_export (1, 1) logical = true;
        optional.fig (1, 1) matlab.ui.Figure = figure("Visible", "on");
        optional.export_fig_cfg (1, 1) ExportFigConfig = ExportFigConfig.paper();
    end

    scenario = create_scenario(options);
    scenario.plot(options, fig = optional.fig);

    set_figure_properties(optional.fig, ExportFigConfig.paper("paperheight", 6))

    %export figure
    if optional.do_export
        folder_path = FileNameConstructor.experiment_result_folder_path( ...
            options ...
        );
        file_name = 'scenario.pdf';
        export_fig(optional.fig, fullfile(folder_path, file_name));
    end

    if (~optional.fig.Visible); close(optional.fig); end

end
