function plot_trajectories(results, options)

    arguments
        results (1, 1) struct; % TODO object; TODO (variable, but clear) array meaning
        options.do_export (1, 1) logical = true;
        options.fig (1, 1) matlab.ui.Figure = figure("Visible", "on");
        options.export_fig_cfg (1, 1) ExportFigConfig = ExportFigConfig.paper();
    end

    set(0, 'currentfigure', options.fig);

    iteration_structs = [results.iteration_structs{:}];
    pose_and_trim = [iteration_structs.x0];
    x = pose_and_trim(:, 1:4:end);
    y = pose_and_trim(:, 2:4:end);
    hold on;
    axis equal;

    % TODO legend
    for i_vehicle = 1:size(x, 1)
        plot(x(i_vehicle, :), y(i_vehicle, :));
    end

    set_figure_properties(options.fig, options.export_fig_cfg);

    %export figure
    if options.do_export
        folder_path = FileNameConstructor.gen_results_folder_path(results(1, 1).scenario.options);
        filepath = fullfile(folder_path, 'trajectories.pdf');
        export_fig(options.fig, filepath);
    end

    if (~options.fig.Visible); close(options.fig); end

end
