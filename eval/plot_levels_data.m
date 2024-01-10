function plot_levels_data(data)
    %   PLOT_LEVELS_DATA     Export plots for computation level evaluation using pre-calculated data

    % sort data
    % Reorder. Assume data is in order [fca, rand, const, color]
    data_permutation = [1 4 2 3];

    nLevels_by_veh_pri = data.nLevels_by_veh_pri;
    nLevels_by_pri = data.nLevels_by_pri;
    nVeh_list = data.nVeh_list;
    result = data.result;
    nPri = data.nPri;

    max_level = max(cellfun(@max, nLevels_by_pri));
    minVeh = min(nVeh_list);
    maxVeh = max(nVeh_list);

    if (maxVeh <= 1)
        warning("A levels plot with only one vehicle is pointless.");
        return;
    end

    % fill rows with zeros
    nLevels_by_pri_mat = [];
    max_entries = max(cellfun(@length, nLevels_by_pri));
    percent_steps_per_level = zeros(max_level, nPri);
    n_levels_med = cellfun(@median, nLevels_by_pri);
    n_levels_max = cellfun(@max, nLevels_by_pri);
    n_levels_med_veh = cellfun(@median, nLevels_by_veh_pri);
    n_levels_max_veh = cellfun(@max, nLevels_by_veh_pri, UniformOutput = false);
    empty_cells = cellfun('isempty', n_levels_max_veh);
    n_levels_max_veh(empty_cells) = {nan};
    n_levels_max_veh = cell2mat(n_levels_max_veh);
    bar_data = [n_levels_med, n_levels_max];

    for iPri = 1:nPri
        n_steps_per_level = histcounts(nLevels_by_pri{iPri}, 1:max_level + 1);
        percent_steps_per_level(:, iPri) = n_steps_per_level / length(nLevels_by_pri{iPri});
        tmp = nLevels_by_pri{iPri};
        tmp(end + 1:max_entries) = 0;
        nLevels_by_pri{iPri} = tmp;
        nLevels_by_pri_mat = [nLevels_by_pri_mat; nLevels_by_pri{iPri}]; %#ok<AGROW>
    end

    % plot
    figHandle = figure();
    barh(1:max_level, percent_steps_per_level);
    set(gca, 'Ydir', 'reverse');
    legend( ...
        '$p_{\mathrm{fca}}$', ...
        '$p_{\mathrm{rand}}$', ...
        '$p_{\mathrm{const}}$', ...
        '$p_{\mathrm{color}}$', ...
        'Location', 'best' ...
    );
    xlabel('Occurences from all steps [\%]', 'Interpreter', 'latex');
    ylabel('$N_{\mathrm{CL}}$', 'Interpreter', 'latex');

    % Export
    folder_path = FileNameConstructor.gen_results_folder_path( ...
        result.options ...
    );
    filename = 'computation-levels-detail.pdf';
    set_figure_properties(figHandle, ExportFigConfig.paper('paperheight', 7))
    export_fig(figHandle, fullfile(folder_path, filename));
    close(figHandle);

    % plot
    figHandle = figure();
    bar_data = bar_data(data_permutation, :);
    barh(1:nPri, bar_data);
    set(gca, 'Ydir', 'reverse');
    y_axis_handle = get(gca, 'YAxis');
    y_axis_handle.TickLabelInterpreter = 'latex';
    xlabel('$N_{\mathrm{CL}}$', 'Interpreter', 'latex');
    ylabel('Priority Assignment Function');
    yticklabels({ ...
                     '$p_{\mathrm{fca}}$', ...
                     '$p_{\mathrm{color}}$', ...
                     '$p_{\mathrm{rand}}$', ...
                     '$p_{\mathrm{const}}$', ...
                 });

    legend( ...
        'median', ...
        'max', ...
        'Location', 'best' ...
    );
    xlim([0, max_level + 1])
    xticks([2, 4, 6])

    % Export
    folder_path = FileNameConstructor.gen_results_folder_path( ...
        result.options ...
    );
    filename = 'computation-levels-overview.pdf';
    set_figure_properties(figHandle, ExportFigConfig.paper('paperheight', 6))
    export_fig(figHandle, fullfile(folder_path, filename));
    close(figHandle);

    % plot
    figHandle = figure();
    markers = {'o', 'x', 'd', '+'
               '^', 'pentagram', '*', 's'};
    colors = ["#0072BD", "#D95319", "#7E2F8E", "#77AC30"];

    for iPri = 1:nPri
        plot(nVeh_list, n_levels_med_veh(:, iPri), 'Marker', markers{1, iPri}, 'Color', colors(iPri), 'LineStyle', 'none', 'MarkerSize', 1)
        hold on
        plot(nVeh_list, n_levels_max_veh(:, iPri), 'Marker', markers{2, iPri}, 'Color', colors(iPri), 'LineStyle', 'none', 'MarkerSize', 1)
    end

    yticks(1:max_level)
    xticks(minVeh:maxVeh)
    ylabel('$N_{\mathrm{CL}}$', 'Interpreter', 'latex');
    xlabel('$N_{A}$', 'Interpreter', 'latex');
    legend( ...
        'median: FCA', ...
        'max: FCA', ...
        'median: Random', ...
        'max: Random', ...
        'median: Constant', ...
        'max: Constant', ...
        'median: Coloring', ...
        'max: Coloring', ...
        'Location', 'best' ...
    );

    % Export
    folder_path = FileNameConstructor.gen_results_folder_path( ...
        result.options ...
    );
    filename = 'computation-levels-graph.pdf';
    % legend currently might cover graph
    set_figure_properties(figHandle, ExportFigConfig.paper('paperheight', 15, 'paperwidth', 15))
    export_fig(figHandle, fullfile(folder_path, filename));
    close(figHandle);

    % plot
    figHandle = figure();
    bar(n_levels_max_veh(:, data_permutation), 'EdgeColor', 'none')
    hold on
    bar(n_levels_med_veh(:, data_permutation), 'EdgeColor', 'none')
    yticks(1:max_level)
    xticks(minVeh:maxVeh)
    ylabel('$N_{\mathrm{CL}}$', 'Interpreter', 'latex');
    xlabel('$N_{A}$', 'Interpreter', 'latex');
    legend( ...
        '$p_{\mathrm{fca}}$ max', ...
        '$p_{\mathrm{color}}$ max', ...
        '$p_{\mathrm{rand}}$ max', ...
        '$p_{\mathrm{const}}$ max', ...
        '$p_{\mathrm{fca}}$ med', ...
        '$p_{\mathrm{color}}$ med', ...
        '$p_{\mathrm{rand}}$ med', ...
        '$p_{\mathrm{const}}$ med', ...
        'Location', 'best', ...
        'NumColumns', 2, ...
        'Interpreter', 'latex' ...
    );
    set(gca, 'TickLength', [0.0025 0.035])
    ylim([0, max_level + 0.5])

    xlim([1.5, maxVeh + 0.5])

    % Export document presets
    folder_path = FileNameConstructor.gen_results_folder_path( ...
        result.options ...
    );
    filename = 'computation-levels-bar-plot-document.pdf';
    set_figure_properties(figHandle, ExportFigConfig.document('paperheight', 6))
    % overwrite color order
    r100 = rwth_color_order;
    r50 = rwth_color_order_50;
    colororder(figHandle, [r50(1:nPri, :); r100(1:nPri, :)]);
    export_fig(figHandle, fullfile(folder_path, filename));
    close(figHandle);

    %% plot computation level reduction
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    n_levels_max_color = n_levels_max_veh(:, 4);
    n_levels_max_other = n_levels_max_veh(:, 1:3);
    level_reduction_percent = 100 * (1 - n_levels_max_color ./ n_levels_max_other);
    n_vehicles = data.nVeh;

    figHandle = figure();
    hold on;
    plot( ...
        1:n_vehicles, level_reduction_percent(:, 3), ...
        'Marker', 's', 'LineStyle', ':' ...
    );
    plot( ...
        1:n_vehicles, level_reduction_percent(:, 2), ...
        'Marker', 'x', 'LineStyle', ':' ...
    );
    plot( ...
        1:n_vehicles, level_reduction_percent(:, 1), ...
        'Marker', '+', 'LineStyle', ':' ...
    );
    hold off;
    ylabel('CL Reduction [\%]', 'Interpreter', 'latex');
    xlabel('$N_{A}$', 'Interpreter', 'latex');
    legend( ...
        '$p_{\mathrm{const}}$', ...
        '$p_{\mathrm{rand}}$', ...
        '$p_{\mathrm{fca}}$', ...
        'Location', 'best', ...
        'Interpreter', 'latex' ...
    );
    xlim([1.5, maxVeh + 0.5])

    % Export document presets
    folder_path = FileNameConstructor.gen_results_folder_path( ...
        result.options ...
    );
    filename = 'computation-levels-reduction.pdf';
    set_figure_properties(figHandle, ExportFigConfig.paper());
    export_fig(figHandle, fullfile(folder_path, filename));
    close(figHandle);
end
