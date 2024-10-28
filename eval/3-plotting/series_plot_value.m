function series_plot_value( ...
        experiment_results ...
        , value ...
        , approach_names ...
        , optional ...
    )

    arguments
        % (n_vehicles x n_approaches x n_scenarios)
        experiment_results (:, :, :) ExperimentResult
        % (n_vehicles x n_approaches)
        value (:, :) double
        approach_names (1, :) string
        optional.ylabel (1, 1) string = "$J_p^{(\cdot)} / J_{p_\mathrm{constant}}^{(\cdot)}$ [\%]";
    end

    % Plot cost
    n_vehicles = [experiment_results(:, 1, 1).n_hlc];
    bar(n_vehicles, value);
    % legend
    legendtext = approach_names;
    legend(legendtext, Location = 'northoutside', NumColumns = 3, Interpreter = 'latex');
    % axes
    xlabel("$N_A$")
    ylabel( ...
        optional.ylabel, ...
        Interpreter = "latex" ...
    );

end
