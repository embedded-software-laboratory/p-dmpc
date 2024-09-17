function [n_levels_min_approach_vehicle, n_levels_med_approach_vehicle, n_levels_mean_approach_vehicle, n_levels_max_approach_vehicle] = data_n_levels_approach_vehicle(experiment_results)

    arguments (Input)
        % ExperimentResult in order (n_vehicles x n_approaches x n_scenarios)
        experiment_results (:, :, :) ExperimentResult
    end

    [n_vehicles, n_approaches, n_scenarios] = size(experiment_results);

    n_levels_approach_vehicle = cell( ...
        n_approaches, ...
        n_vehicles ...
    );

    [n_levels_approach_vehicle{:}] = deal([]);

    for i_vehicles = 1:n_vehicles

        for i_approaches = 1:n_approaches

            for i_scenarios = 1:n_scenarios

                experiment_result = experiment_results(i_vehicles, i_approaches, i_scenarios);

                if experiment_result.hlc_indices == -1
                    continue;
                end

                for j_vehicle = 1:experiment_result.options.amount
                    % get all n_levels in experiment as row vector
                    n_levels_experiment = reshape( ...
                        [experiment_result.iteration_data(:, :).number_of_computation_levels], ...
                        1, ...
                        numel(experiment_result.iteration_data(:, :)) ...
                    );
                    % insert values into the correct list
                    n_levels_approach_vehicle{i_approaches, i_vehicles} = [ ...
                                                                               n_levels_approach_vehicle{i_approaches, i_vehicles}, ...
                                                                               n_levels_experiment ...
                                                                           ];

                end

            end

        end

    end

    n_levels_approach_vehicle_empty_cells = cellfun(@isempty, n_levels_approach_vehicle);
    [n_levels_approach_vehicle{n_levels_approach_vehicle_empty_cells}] = deal([0]);

    % extract information
    n_levels_min_approach_vehicle = cellfun(@min, n_levels_approach_vehicle);
    n_levels_med_approach_vehicle = cellfun(@median, n_levels_approach_vehicle);
    n_levels_mean_approach_vehicle = cellfun(@mean, n_levels_approach_vehicle);
    n_levels_max_approach_vehicle = cellfun(@max, n_levels_approach_vehicle);

end
