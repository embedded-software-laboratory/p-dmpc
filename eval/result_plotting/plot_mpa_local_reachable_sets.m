function plot_mpa_local_reachable_sets(mpa, options, optional)

    arguments
        mpa (1, 1) MotionPrimitiveAutomaton;
        options (1, 1) Config
        optional.do_export (1, 1) logical = true;
        optional.fig (1, 1) matlab.ui.Figure = figure(Visible = "on");
        optional.export_fig_cfg (1, 1) ExportFigConfig = ExportFigConfig.paper();
    end

    mpa.plot_local_reachable_sets(fig = optional.fig);

    set_figure_properties(optional.fig, optional.export_fig_cfg);

    %export figure
    if optional.do_export
        file_ext = '.pdf';
        folder_path = FileNameConstructor.experiment_result_folder_path(options);
        [~, file_name, ~] = fileparts(FileNameConstructor.get_mpa_name(options));
        filepath = fullfile(folder_path, [file_name, '_local_reachable_sets', file_ext]);
        export_fig(optional.fig, filepath);
    end

    if (~optional.fig.Visible); close(optional.fig); end

end
