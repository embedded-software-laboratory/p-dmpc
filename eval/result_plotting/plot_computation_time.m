function plot_computation_time(results, options)

    arguments
        results (:, :) struct; % TODO object; TODO (variable, but clear) array meaning
        options.do_export (1, 1) logical = true;
        options.fig (1, 1) matlab.ui.Figure = figure("Visible", "on");
        options.export_fig_cfg (1, 1) ExportFigConfig = ExportFigConfig.paper();
    end

    nResults = length(results);

    runtimes = [];
    nVehicles = [];

    fig_per_result = figure(visible = "off");
    set(0, 'currentfigure', fig_per_result);

    for i = 1:nResults
        % make sure only one controller runtime is stored in the struct
        % assert(size(results(i).controller_runtime, 1) == 1); % TODO
        runtimes = [runtimes, results(i).timings_general.controller.'];
        nVehicles = [nVehicles, results(i).scenario.options.amount];

        bar(1:numel(runtimes), runtimes);
        % set labels
        xlabel('Time step $k$', 'Interpreter', 'LaTex');
        ylabel('Computation Time [s]', 'Interpreter', 'LaTex');

        if options.do_export

            results_folder = FileNameConstructor.gen_results_folder_path(results(i).scenario.options);
            filepath = fullfile(results_folder, sprintf('computation_time_result_%d.pdf', i));
            set_figure_properties(fig_per_result, ExportFigConfig.paper('paperheight', 12))
            export_fig(fig_per_result, filepath);
            if (~fig_per_result.Visible); close(fig_per_result); end
        end

    end

    [~, order] = sort(nVehicles);

    runtimes(:, :) = runtimes(:, order);
    nVehicles(:) = nVehicles(order);

    % plot with axis on both sides and logarithmic scale
    set(0, 'currentfigure', options.fig);
    boxplot(runtimes, nVehicles, 'MedianStyle', 'line');
    set(gca, 'YScale', 'log');

    % set labels
    xlabel('Number of Vehicles', 'Interpreter', 'LaTex');
    ylabel('Computation Time [s]', 'Interpreter', 'LaTex');

    set_figure_properties(options.fig, ExportFigConfig.paper('paperheight', 12))

    if options.do_export
        results_folder = FileNameConstructor.all_results();
        filepath = fullfile(results_folder, 'computation_time.pdf');
        export_fig(options.fig, filepath);
    end

    if (~options.fig.Visible); close(options.fig); end

end
