function plot_mpa_over_time(mpa, scenario, options)

    arguments
        mpa (1, 1) MotionPrimitiveAutomaton;
        scenario (1, 1) Scenario;
        options.y_lim (1, 2) double = [-0.1, 1.1];
        options.do_export (1, 1) logical = false;
        options.fig (1, 1) matlab.ui.Figure = figure(Visible = "On");
        options.export_fig_cfg (1, 1) ExportFigConfig = ExportFigConfig.paper();
    end

    mpa.plot_over_time(y_lim = options.y_lim);

    %export figure
    if options.do_export
        options.fig.Visible = "off";
        file_ext = '.pdf';
        folder_path = FileNameConstructor.gen_results_folder_path(scenario.options);
        [~, file_name, ~] = fileparts(FileNameConstructor.get_mpa_name(scenario.options));
        filepath = fullfile(folder_path, [file_name file_ext]);
        set_figure_properties(options.fig, options.export_fig_cfg);
        export_fig(options.fig, filepath)
        close(options.fig);
    end

end
