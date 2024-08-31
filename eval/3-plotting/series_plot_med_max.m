function series_plot_med_max( ...
        experiment_results ...
        , median_value ...
        , maximum_value ...
        , approach_names ...
        , optional ...
    )

    arguments
        % (n_vehicles x n_approaches x n_scenarios)
        experiment_results (:, :, :) ExperimentResult
        % (n_vehicles x n_approaches)
        median_value (:, :) double
        % (n_vehicles x n_approaches)
        maximum_value (:, :) double
        approach_names (:, 1) string
        optional.ylabel (1, 1) string = "$T_{\mathrm{NCS}}$ [ms]"
        optional.export_fig_config (1, 1) ExportFigConfig = ExportFigConfig.paper()
    end

    n_vehicles = [experiment_results(:, 1, 1).n_hlc];
    fig = gcf;
    max_bar = bar(n_vehicles, maximum_value);
    hold_before = ishold();
    hold on;
    med_bar = bar(n_vehicles, median_value);

    if ~hold_before
        hold off;
    end

    % legend
    str_med = "med ";
    str_max = "max ";
    legendtext = [ ...
                      strcat(repmat(str_med, length(approach_names), 1), approach_names) ...
                      strcat(repmat(str_max, length(approach_names), 1), approach_names) ...
                  ];
    legend([med_bar, max_bar], legendtext, Location = 'northoutside', Interpreter = 'latex', NumColumns = 2);
    % axes
    xlabel('$N_{A}$', Interpreter = 'latex');
    ylabel(optional.ylabel, Interpreter = 'latex');

    set_figure_properties(fig, optional.export_fig_config);
    rwth_colors_100 = rwth_color_order;
    rwth_colors_50 = rwth_color_order_50;
    colororder( ...
        fig, ...
        [rwth_colors_50(1:length(approach_names), :); ...
         rwth_colors_100(1:length(approach_names), :)] ...
    );
end
