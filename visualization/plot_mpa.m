function plot_mpa(scenario,options)
arguments
    scenario            (1,1) Scenario;
    options.y_lim       (1,2) double = [-0.1, 1.0];
    options.x_lim       (1,2) double = rad2deg(pi/18*[-3, 3]);
    options.k           (1,1) double = 1;
    options.do_export   (1,1) logical = false;
    options.fig         (1,1) matlab.ui.Figure = figure("Visible","off");
    options.with_labels (1,1) logical = true;
    options.export_fig_cfg (1,1) ExportFigConfig = ExportFigConfig.document();
end
    mpa = scenario.mpa;
    
    trim_inputs = mpa.trims;

    trim_adjacency = mpa.transition_matrix_single(:,:,options.k);
    
    angle = rad2deg([trim_inputs.steering]);
    speed = [trim_inputs.speed];
    G = digraph(trim_adjacency,'omitSelfLoops');
    
    plot(G,'XData',angle,'YData',speed, ...
        'ArrowSize', 5, ...
        'MarkerSize', 3, ...
        'NodeColor', rwth_blue(), ...
        'EdgeColor', rwth_blue_50(), ...
        'EdgeAlpha',1 ...
    );

    if options.with_labels
        xlabel('Steering Angle $\delta$ [$^{\circ}$]','Interpreter','latex');
        ylabel('Speed $\mathrm{v}$ [m/s]','Interpreter','latex');
    end
    
    if isfield(options, 'x_lim')
        xlim(options.x_lim);
    end
    if isfield(options, 'y_lim')
        ylim(options.y_lim);
    end
    grid on

    if options.do_export
        file_ext = '.pdf';
        folder_path = FileNameConstructor.gen_results_folder_path(scenario.options);
        [~,file_name,~] = fileparts(FileNameConstructor.get_mpa_name(scenario.options));
        filepath = fullfile(folder_path,[file_name file_ext]);
        set_figure_properties(options.fig,options.export_fig_cfg);
        export_fig(options.fig, filepath)
        close(options.fig);
    else
        options.fig.Visible = 'on';
    end
end