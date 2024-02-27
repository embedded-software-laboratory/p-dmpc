function [time_min_approach_vehicle, time_med_approach_vehicle, time_mean_approach_vehicle, time_max_approach_vehicle] = data_time_approach_vehicle(experiment_results)

    arguments (Input)
        % ExperimentResult in order (n_vehicles x n_approaches x n_scenarios)
        experiment_results (:, :, :) ExperimentResult
    end

    [n_vehicles, n_approaches, n_scenarios] = size(experiment_results);

    times_approach_vehicle = cell( ...
        n_approaches, ...
        n_vehicles ...
    );

    [times_approach_vehicle{:}] = deal([]);

    for i_vehicles = 1:n_vehicles

        for i_approaches = 1:n_approaches

            for i_scenarios = 1:n_scenarios

                experiment_result = experiment_results(i_vehicles, i_approaches, i_scenarios);

                for j_vehicle = 1:experiment_result.options.amount
                    % get all control loop times as row vector
                    times_experiment = reshape( ...
                        experiment_result.timing(j_vehicle).control_loop(2, :), ...
                        1, ...
                        length(experiment_result.timing(j_vehicle).control_loop(2, :)) ...
                    );
                    % insert timings into the correct list
                    times_approach_vehicle{i_approaches, i_vehicles} = [ ...
                                                                            times_approach_vehicle{i_approaches, i_vehicles}, ...
                                                                            times_experiment ...
                                                                        ];

                end

            end

        end

    end

    % extract information
    time_min_approach_vehicle = cellfun(@min, times_approach_vehicle);
    time_med_approach_vehicle = cellfun(@median, times_approach_vehicle);
    time_mean_approach_vehicle = cellfun(@mean, times_approach_vehicle);
    time_max_approach_vehicle = cellfun(@max, times_approach_vehicle);

end
