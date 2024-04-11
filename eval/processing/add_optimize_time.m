function computation_time = add_optimize_time(experiment_result, computation_time)

    arguments (Input)
        experiment_result (1, 1) ExperimentResult
        computation_time (:, :) double
    end

    arguments (Output)
        % n_vehicles x n_steps
        computation_time (:, :) double
    end

    n_vehicles = experiment_result.n_hlc;

    all_field_names = fieldnames(experiment_result.timing(1));
    optimize_field_names_indices = ~cellfun(@isempty, regexp(all_field_names, '^optimize\w+'));
    optimize_field_names = string(all_field_names(optimize_field_names_indices))';

    % n_vehicles x n_steps x n_permutations
    optimize_start = zeros([size(computation_time), numel(optimize_field_names)]);
    optimize_duration = zeros(size(optimize_start));

    for i_field = 1:numel(optimize_field_names)
        optimize_timing = vertcat(experiment_result.timing.(optimize_field_names(i_field)));
        optimize_start(:, :, i_field) = optimize_timing(1:2:end, :);
        optimize_duration(:, :, i_field) = optimize_timing(2:2:end, :);
    end

    for i_step = 1:experiment_result.n_steps

        % Create directed_coupling_sequential for each optimize permutation
        % Current number of levels: nnz of optimize field duration
        permutations = squeeze(optimize_duration(1, i_step, :) ~= 0);
        n_permutations = nnz(permutations);
        levels_per_vehicle = zeros(n_vehicles, n_permutations);
        directed_coupling_sequential = false([n_vehicles, n_vehicles, n_permutations]);

        if n_permutations == 1
            directed_coupling_sequential(:, :, 1) = experiment_result.iteration_data(i_step).directed_coupling_sequential;
            levels_per_vehicle(:, 1) = kahn( ...
                directed_coupling_sequential(:, :, 1) ...
            );

        else

            % latin_square: n_vehicles x n_permutations
            % columns are levels
            % entry in row indicates which permutation the vehicle solved at
            % level
            [~, latin_square] = sort(squeeze(optimize_start(:, i_step, permutations)), 2);

            adjacency_sequential = experiment_result.iteration_data(i_step).directed_coupling_sequential ...
                + experiment_result.iteration_data(i_step).directed_coupling_sequential';

            for i_perm = 1:n_permutations
                % level_matrix: n_vehicles x n_levels
                level_matrix = latin_square == i_perm;
                priorities = level_matrix * (1:n_permutations)';
                directed_coupling_sequential(:, :, i_perm) = Prioritizer.directed_coupling_from_priorities( ...
                    adjacency_sequential, ...
                    priorities ...
                );

                levels_per_vehicle(:, i_perm) = kahn(directed_coupling_sequential(:, :, i_perm));
            end

        end

        for i_level = 1:max(levels_per_vehicle)
            computation_time_from_level = zeros(size(computation_time(:, i_step)));
            % Loop over permutations
            for i_perm = 1:n_permutations

                vehicles_on_level = find(levels_per_vehicle(:, i_perm) == i_level);
                vehicles_on_level = reshape(vehicles_on_level, 1, []);

                for i_vehicle = vehicles_on_level
                    predecessors = directed_coupling_sequential(:, i_vehicle, i_perm);
                    computation_time_from_level(i_vehicle) = ...
                        max([ ...
                             computation_time(i_vehicle, i_step), ...
                             computation_time(predecessors, i_step)' ...
                         ]) + optimize_duration(i_vehicle, i_step, i_perm);
                end

            end

            computation_time(:, i_step) = computation_time(:, i_step) + computation_time_from_level;
        end

    end

end
