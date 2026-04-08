function T_communication = data_time_communication_experiment(experiment_result)

    arguments (Input)
        experiment_result (1, 1) ExperimentResult
    end

    arguments (Output)
        T_communication (:, 1) double
    end

    T_communication = []; % array of all communication durations

    if experiment_result.options.priority == PriorityStrategies.optimal_priority
        % Optimal priority returns negative communication duration
        % Reason unclear
        return
    end

    n_vehicles = experiment_result.n_hlc;

    all_field_names = fieldnames(experiment_result.timing(1));
    optimize_field_names_indices = ~cellfun(@isempty, regexp(all_field_names, '^optimize\w+'));
    % FIXME check if ordering can be problematic elsewhere
    optimize_field_names = strcat("optimize", string(0:nnz(optimize_field_names_indices) - 1)');

    % n_vehicles x n_steps x n_permutations
    optimize_start = zeros([n_vehicles, experiment_result.n_steps, numel(optimize_field_names)]);
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
        directed_coupling_sequential = false([n_vehicles, n_vehicles, n_permutations]);

        if n_permutations == 1
            directed_coupling_sequential(:, :, 1) = experiment_result.iteration_data(i_step).directed_coupling_sequential;
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
            end

        end

        % Loop over permutations
        for i_perm = 1:n_permutations

            for i_vehicle = 1:n_vehicles
                predecessors = directed_coupling_sequential(:, i_vehicle, i_perm);

                if ~any(predecessors)
                    continue;
                end

                t_end_optimization = ...
                    optimize_start(predecessors, i_step, i_perm) ...
                    + optimize_duration(predecessors, i_step, i_perm);
                t_max_predecessors = max(t_end_optimization);

                if n_permutations == 1;
                    t_end_optimization_self = 0;
                else
                    % From previous permutation
                    [~, latin_square] = sort(squeeze(optimize_start(:, i_step, permutations)), 2);
                    i_level = find(latin_square(i_vehicle, :) == i_perm);
                    i_previous_perm = latin_square(i_vehicle, i_level - 1);

                    t_end_optimization_self = ...
                        optimize_start(i_vehicle, i_step, i_previous_perm) ...
                        + optimize_duration(i_vehicle, i_step, i_previous_perm);

                end

                t_could_optimize_start = max( ...
                    t_max_predecessors, ...
                    t_end_optimization_self ...
                );

                T_communication(end + 1) = ...
                    optimize_start(i_vehicle, i_step, i_perm) ...
                    - t_could_optimize_start; %#ok<AGROW>

                if T_communication(end) > 0.250
                    continue;
                end

            end

        end

    end

end
