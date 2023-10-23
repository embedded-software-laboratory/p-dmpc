function plot_mpa_over_time(mpa, scenario, options)

    arguments
        mpa (1, 1) MotionPrimitiveAutomaton;
        scenario (1, 1) Scenario;
        options.y_lim (1, 2) double = [-0.1, 1.1];
        options.do_export (1, 1) logical = true;
        options.fig (1, 1) matlab.ui.Figure = figure(Visible = "on");
        options.export_fig_cfg (1, 1) ExportFigConfig = ExportFigConfig.paper();
    end

    cleanupObj = onCleanup(@()end_plot_mpa_over_time(options.fig, options.do_export));

    mpa.plot_over_time(y_lim = options.y_lim, fig = options.fig);

    %export figure
    if options.do_export
        file_ext = '.pdf';
        folder_path = FileNameConstructor.gen_results_folder_path(scenario.options);
        [~, file_name, ~] = fileparts(FileNameConstructor.get_mpa_name(scenario.options));
        filepath = fullfile(folder_path, [file_name, '_over_time', file_ext]);
        set_figure_properties(options.fig, options.export_fig_cfg);
        export_fig(options.fig, filepath)
    end

end

function end_plot_mpa_over_time(fig, do_export)

    arguments
        fig (1, 1) matlab.ui.Figure;
        do_export (1, 1) logical
    end

    if do_export
        close(fig);
    end

end
