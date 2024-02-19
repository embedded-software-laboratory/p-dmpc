function cost_percent_average = data_cost_percent(experiment_results)
arguments
    % ExperimentResult in order (n_vehicles x n_approaches x n_scenarios)
    % Last approach is assumed to be optimal approach
    experiment_results (:, :, :)ExperimentResult
end

cost = zeros(size(experiment_results));

for i = 1:numel(experiment_results)
    cost(i) = data_cost_experiment(experiment_results(i));
end

optimal_cost = cost(:, end, :);
cost_increase = cost(:, 1:end - 1, :) ./ optimal_cost;
cost_percent = cost_increase * 100;
cost_percent_average = mean(cost_percent, 3);
end