function plot_computation_time_per_vehicle(experiment_result, optional)

    arguments
        experiment_result (1, 1) ExperimentResult;
        optional.do_export (1, 1) logical = true;
        optional.fig (1, 1) matlab.ui.Figure = figure(visible = "off");
        optional.export_fig_cgf (1, 1) ExportFigConfig = ExportFigConfig.paper();
    end

    n_results = size(experiment_result.n_expanded, 1);

    runtimes = zeros(experiment_result.n_steps, n_results);

    for i = 1:n_results
        optional.fig = figure(visible = "off");
        set(0, 'currentfigure', optional.fig);
        % make sure only one controller runtime is stored in the struct
        % assert(size(experiment_result(i).controller_runtime, 1) == 1); % TODO
        runtimes(:, i) = squeeze(experiment_result.timing(i).control_loop(2, 1, :));

        bar(1:size(runtimes, 1), runtimes);
        % set labels
        xlabel('Time step $k$', 'Interpreter', 'LaTex');
        ylabel('Computation Time [s]', 'Interpreter', 'LaTex');

        if optional.do_export

            results_folder = FileNameConstructor.gen_results_folder_path(experiment_result.options);
            filepath = fullfile(results_folder, sprintf('computation_time_result_%d.pdf', i));
            set_figure_properties(optional.fig, ExportFigConfig.paper('paperheight', 12))
            export_fig(optional.fig, filepath);
            if (~optional.fig.Visible); close(optional.fig); end
        end

    end

end
