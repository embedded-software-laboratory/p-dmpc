function plot_mpa_local_reachable_sets(mpa, scenario, options)

    arguments
        mpa (1, 1) MotionPrimitiveAutomaton;
        scenario (1, 1) Scenario;
        options.do_export (1, 1) logical = true;
        options.fig (1, 1) matlab.ui.Figure = figure(Visible = "on");
        options.export_fig_cfg (1, 1) ExportFigConfig = ExportFigConfig.paper();
    end

    mpa.plot_local_reachable_sets(fig = options.fig);

    set_figure_properties(options.fig, options.export_fig_cfg);

    %export figure
    if options.do_export
        file_ext = '.pdf';
        folder_path = FileNameConstructor.gen_results_folder_path(scenario.options);
        [~, file_name, ~] = fileparts(FileNameConstructor.get_mpa_name(scenario.options));
        filepath = fullfile(folder_path, [file_name, '_local_reachable_sets', file_ext]);
        export_fig(options.fig, filepath);
    end

    if (~options.fig.Visible); close(options.fig); end

end
