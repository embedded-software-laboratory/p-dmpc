function series_plot_cost_percent( ...
    experiment_results ...
    , cost_percent ...
    , approach_names ...
)

    arguments
        % (n_vehicles x n_approaches x n_scenarios)
        experiment_results (:, :, :) ExperimentResult
        % (n_vehicles x n_approaches)
        cost_percent (:, :) double
        approach_names (1, :) string
    end

    % Plot cost
    n_vehicles = [experiment_results(:, 1, 1).n_hlc];
    bar(n_vehicles, cost_percent);
    % legend
    legendtext = approach_names;
    legend(legendtext, Location = 'northoutside', NumColumns = 3, Interpreter = 'latex');
    % axes
    xlabel("$N_A$")
    ylabel( ...
        "$J_\mathrm{NCS}(p) / J_\mathrm{NCS}(p_\mathrm{constant})$ [\%]", ...
        Interpreter = "latex" ...
    );

end
