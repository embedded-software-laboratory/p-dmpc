function plot_mpa(mpa, scenario, options)

    arguments
        mpa (1, 1) MotionPrimitiveAutomaton;
        scenario (1, 1) Scenario;
        options.y_lim (1, 2) double = [-0.1, 1.0];
        options.x_lim (1, 2) double = rad2deg(pi / 18 * [-3, 3]);
        options.k (1, 1) double = 1;
        options.with_labels (1, 1) logical = true;
        options.do_export (1, 1) logical = true;
        options.fig (1, 1) matlab.ui.Figure = figure("Visible", "on");
        options.export_fig_cfg (1, 1) ExportFigConfig = ExportFigConfig.paper();
    end

    cleanupObj = onCleanup(@()end_plot_mpa(options.fig, options.do_export));

    mpa.plot( ...
        y_lim = options.y_lim, ...
        x_lim = options.x_lim, ...
        k = options.k, ...
        fig = options.fig, ...
        with_labels = options.with_labels ...
    );

    %export figure
    if options.do_export
        file_ext = '.pdf';
        folder_path = FileNameConstructor.gen_results_folder_path(scenario.options);
        [~, file_name, ~] = fileparts(FileNameConstructor.get_mpa_name(scenario.options));
        filepath = fullfile(folder_path, [file_name file_ext]);
        set_figure_properties(options.fig, options.export_fig_cfg);
        export_fig(options.fig, filepath)
    end

end

function end_plot_mpa(fig, do_export)

    arguments
        fig (1, 1) matlab.ui.Figure;
        do_export (1, 1) logical
    end

    if do_export
        close(fig);
    end

end
