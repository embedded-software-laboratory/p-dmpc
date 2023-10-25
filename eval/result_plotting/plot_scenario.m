function plot_scenario(scenario, options)

    arguments
        scenario (1, 1) Scenario;
        options.do_export (1, 1) logical = true;
        options.fig (1, 1) matlab.ui.Figure = figure("Visible", "on");
        options.export_fig_cfg (1, 1) ExportFigConfig = ExportFigConfig.paper();
    end

    scenario.plot(fig = options.fig);

    %export figure
    if options.do_export
        folder_path = FileNameConstructor.gen_results_folder_path( ...
            scenario.options ...
        );
        file_name = 'scenario.pdf';
        set_figure_properties(options.fig, ExportFigConfig.paper("paperheight", 6))

        export_fig(options.fig, fullfile(folder_path, file_name));
        if (~options.fig.Visible); close(options.fig); end
    end

end
