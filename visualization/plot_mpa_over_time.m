function plot_mpa_over_time(scenario, mpa, options)

    arguments
        scenario (1, 1) Scenario;
        mpa (1, 1) MotionPrimitiveAutomaton;
        options.y_lim (1, 2) double = [-0.1, 1.1];
        options.do_export (1, 1) logical = false;
        options.export_fig_cfg (1, 1) ExportFigConfig = ExportFigConfig.paper();
    end

    fig = figure("Visible", "off");
    tiledLayoutHandle = tiledlayout( ...
        1, scenario.options.Hp, ...
        'TileSpacing', 'compact', ...
        'Padding', 'compact' ...
    );

    for k = 1:scenario.options.Hp
        nexttile
        plot_mpa(scenario, mpa, ...
            'do_export', false, ...
            'fig', fig, ...
            'k', k, ...
            'with_labels', false, ...
            'x_lim', rad2deg(pi / 18 * [-3, 3]) ...
        );
        title(sprintf("$t=k+%d$", k - 1), 'Interpreter', 'latex');
    end

    xlabel(tiledLayoutHandle, 'Steering Angle $\delta$ [$^{\circ}$]', ...
        'Interpreter', 'latex' ...
    );
    ylabel(tiledLayoutHandle, 'Speed $\mathrm{v}$ [m/s]', ...
        'Interpreter', 'latex' ...
    );

    if options.do_export
        file_ext = '.pdf';
        folder_path = FileNameConstructor.gen_results_folder_path(scenario.options);
        [~, file_name, ~] = fileparts(FileNameConstructor.get_mpa_name(scenario.options));
        filepath = fullfile(folder_path, [file_name file_ext]);
        set_figure_properties(fig, options.export_fig_cfg);
        export_fig(fig, filepath)
        close(fig);
    else
        fig.Visible = 'on';
    end

end
