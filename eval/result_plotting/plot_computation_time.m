function plot_computation_time(results, optional)

    arguments
        results (:, :) struct; % TODO object; TODO (variable, but clear) array meaning
        optional.do_export (1, 1) logical = true;
        optional.fig (1, 1) matlab.ui.Figure = figure("Visible", "on");
        optional.export_fig_cfg (1, 1) ExportFigConfig = ExportFigConfig.paper();
    end

    n_results = length(results);

    runtimes = zeros(results(1).n_steps, n_results);
    n_vehicles = zeros(1, n_results);

    fig_per_result = figure(visible = "off");
    set(0, 'currentfigure', fig_per_result);

    for i = 1:n_results
        % make sure only one controller runtime is stored in the struct
        % assert(size(results(i).controller_runtime, 1) == 1); % TODO
        runtimes(:, i) = squeeze(results(i).timings_general.controller(2, 1, :));
        n_vehicles(i) = results(i).options.amount;

        bar(1:numel(runtimes), runtimes);
        % set labels
        xlabel('Time step $k$', 'Interpreter', 'LaTex');
        ylabel('Computation Time [s]', 'Interpreter', 'LaTex');

        if optional.do_export

            results_folder = FileNameConstructor.gen_results_folder_path(results(i).options);
            filepath = fullfile(results_folder, sprintf('computation_time_result_%d.pdf', i));
            set_figure_properties(fig_per_result, ExportFigConfig.paper('paperheight', 12))
            export_fig(fig_per_result, filepath);
            if (~fig_per_result.Visible); close(fig_per_result); end
        end

    end

    [~, order] = sort(n_vehicles);

    runtimes(:, :) = runtimes(:, order);
    n_Vehicles(:) = n_vehicles(order);

    % plot with axis on both sides and logarithmic scale
    set(0, 'currentfigure', optional.fig);
    boxplot(runtimes, n_Vehicles, 'MedianStyle', 'line');
    set(gca, 'YScale', 'log');

    % set labels
    xlabel('Number of Vehicles', 'Interpreter', 'LaTex');
    ylabel('Computation Time [s]', 'Interpreter', 'LaTex');

    set_figure_properties(optional.fig, ExportFigConfig.paper('paperheight', 12))

    if optional.do_export
        results_folder = FileNameConstructor.all_results();
        filepath = fullfile(results_folder, 'computation_time.pdf');
        export_fig(optional.fig, filepath);
    end

    if (~optional.fig.Visible); close(optional.fig); end

end
