function cost_percent_average = data_cost_percent(experiment_results)

    arguments
        % ExperimentResult in order (n_vehicles x n_approaches x n_scenarios)
        % First approach is assumed to be constant approach
        experiment_results (:, :, :)ExperimentResult
    end

    cost = zeros(size(experiment_results));

    for i = 1:numel(experiment_results)

        if experiment_results(i).hlc_indices == -1
            cost(i) = 0;
            continue;
        end

        cost(i) = data_cost_experiment(experiment_results(i));
    end

    baseline_cost = cost(:, 1, :);
    cost_increase = cost ./ baseline_cost;
    cost_percent = cost_increase * 100;
    cost_percent_average = mean(cost_percent, 3);
end
