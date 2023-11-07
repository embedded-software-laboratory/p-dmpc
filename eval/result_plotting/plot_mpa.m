function plot_mpa(mpa, options, optional)

    arguments
        mpa (1, 1) MotionPrimitiveAutomaton;
        options (1, 1) Config;
        optional.y_lim (1, 2) double = [-0.1, 1.0];
        optional.x_lim (1, 2) double = rad2deg(pi / 5 * [-1, 1]);
        optional.k (1, 1) double = 1;
        optional.with_labels (1, 1) logical = true;
        optional.do_export (1, 1) logical = true;
        optional.fig (1, 1) matlab.ui.Figure = figure("Visible", "on");
        optional.export_fig_cfg (1, 1) ExportFigConfig = ExportFigConfig.paper();
    end

    mpa.plot( ...
        y_lim = optional.y_lim, ...
        x_lim = optional.x_lim, ...
        k = optional.k, ...
        fig = optional.fig, ...
        with_labels = optional.with_labels ...
    );

    set_figure_properties(optional.fig, optional.export_fig_cfg);

    %export figure
    if optional.do_export
        file_ext = '.pdf';
        folder_path = FileNameConstructor.gen_results_folder_path(options);
        [~, file_name, ~] = fileparts(FileNameConstructor.get_mpa_name(options));
        filepath = fullfile(folder_path, [file_name file_ext]);
        export_fig(optional.fig, filepath);
    end

    if (~optional.fig.Visible); close(optional.fig); end

end
