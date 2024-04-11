function [time_min_approach_vehicle, time_med_approach_vehicle, time_mean_approach_vehicle, time_max_approach_vehicle] = data_time_approach_vehicle(experiment_results, optional)

    arguments (Input)
        % ExperimentResult in order (n_vehicles x n_approaches x n_scenarios)
        experiment_results (:, :, :) ExperimentResult
        optional.computation_time_function (1, 1) function_handle = @data_time_experiment
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

                if experiment_result.hlc_indices == -1
                    continue;
                end

                computation_time_per_vehicle = optional.computation_time_function(experiment_result);
                computation_time_ncs = max(computation_time_per_vehicle);

                % insert timings into the correct list
                times_approach_vehicle{i_approaches, i_vehicles} = reshape( ...
                    computation_time_ncs, 1, [] ...
                );

            end

        end

    end

    times_approach_vehicle_empty_cells = cellfun(@isempty, times_approach_vehicle);
    [times_approach_vehicle{times_approach_vehicle_empty_cells}] = deal(0);

    % extract information
    time_min_approach_vehicle = cellfun(@min, times_approach_vehicle);
    time_med_approach_vehicle = cellfun(@median, times_approach_vehicle);
    time_mean_approach_vehicle = cellfun(@mean, times_approach_vehicle);
    time_max_approach_vehicle = cellfun(@max, times_approach_vehicle);

end
