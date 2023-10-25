function plot_trajectories(results, options)

    arguments
        results (:, :) struct; % TODO object; TODO (variable, but clear) array meaning
        options.do_export (1, 1) logical = false;
        options.fig (1, 1) matlab.ui.Figure = figure("Visible", "on");
        options.export_fig_cfg (1, 1) ExportFigConfig = ExportFigConfig.paper();
    end

    set(0, 'currentfigure', options.fig);

    for result = results
        iteration_structs = [result.iteration_structs{:}];
        pose_and_trim = [iteration_structs.x0];
        x = pose_and_trim(:, 1:4:end);
        y = pose_and_trim(:, 2:4:end);
        hold on;
        axis equal;

        % TODO legend
        for i_vehicle = 1:size(x, 1)
            plot(x(i_vehicle, :), y(i_vehicle, :));
        end

    end

    %export figure
    if options.do_export
        filepath = fullfile('results', 'trajectories.pdf');
        set_figure_properties(options.fig, options.export_fig_cfg);
        export_fig(options.fig, filepath);
        if (~options.fig.Visible); close(options.fig); end
    end

end
