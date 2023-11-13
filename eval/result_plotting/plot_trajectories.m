function plot_trajectories(experiment_result, optional)

    arguments
        experiment_result (1, 1) ExperimentResult; % TODO object; TODO (variable, but clear) array meaning
        optional.do_export (1, 1) logical = true;
        optional.fig (1, 1) matlab.ui.Figure = figure("Visible", "on");
        optional.export_fig_cfg (1, 1) ExportFigConfig = ExportFigConfig.paper();
    end

    set(0, 'currentfigure', optional.fig);

    iteration_data = [experiment_result.iteration_data{:}];
    pose_and_trim = [iteration_data.x0];
    x = pose_and_trim(:, 1:4:end);
    y = pose_and_trim(:, 2:4:end);
    hold on;
    axis equal;

    % TODO legend
    for i_vehicle = 1:size(x, 1)
        plot(x(i_vehicle, :), y(i_vehicle, :));
    end

    set_figure_properties(optional.fig, optional.export_fig_cfg);

    %export figure
    if optional.do_export
        folder_path = FileNameConstructor.gen_results_folder_path(experiment_result(1, 1).options);
        filepath = fullfile(folder_path, 'trajectories.pdf');
        export_fig(optional.fig, filepath);
    end

    if (~optional.fig.Visible); close(optional.fig); end

end
