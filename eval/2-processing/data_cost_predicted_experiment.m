function cost = data_cost_predicted_experiment(experiment_result)

    arguments
        experiment_result (1, 1) ExperimentResult
    end

    % cost of the OCP, summed over agents, summed over time steps

    cost = 0;

    n_vehicles = experiment_result.options.amount;
    n_steps = experiment_result.n_steps;

    for i_step = 1:n_steps

        for i_vehicle = 1:n_vehicles
            trajectory_prediction = experiment_result.control_results_info(i_vehicle, i_step).y_predicted(1:2, :);

            trajectory_reference = ...
                experiment_result.iteration_data(i_step).reference_trajectory_points( ...
                i_vehicle, ...
                :, ...
                : ...
            );
            trajectory_reference = squeeze(trajectory_reference)';

            cost = cost + sum(vecnorm(trajectory_reference - trajectory_prediction).^2);
        end

    end

end
