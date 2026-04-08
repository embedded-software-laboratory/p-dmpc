function plot_computation_time_for_step(experiment_result, k, optional)
    %% PLOT_RUNTIME_FOR_STEP plots the timing of the specified timestep where t=0 is the start of the first hlc_init measurement
    %%      IMPORTANT: This function expects a result as input in which the controller_start_times were normalized before
    %%                 by normalize_timing_results

    arguments
        experiment_result (1, 1) ExperimentResult;
        k uint64;
        optional.fig (1, 1) matlab.ui.Figure = gcf;
    end

    hold_before = ishold;

    options = experiment_result.options;

    % Configure, which field names in the timing object are relevant, dependent on the used controller
    if ~(options.is_prioritized && options.computation_mode ~= ComputationMode.sequential)
        error('The graph is currently only supported for results of prioritized, distributed execution.');
    end

    % find computation order for sorted plotting
    i_step = k;
    n_vehicles = experiment_result.n_hlc;

    all_field_names = fieldnames(experiment_result.timing(1));
    optimize_field_names_indices = ~cellfun(@isempty, regexp(all_field_names, '^optimize\w+'));
    optimize_field_names = string(all_field_names(optimize_field_names_indices))';

    % n_vehicles x n_steps x n_permutations
    optimize_start = zeros([n_vehicles, 1, numel(optimize_field_names)]); % 3d because of similarity from add_optimize_time
    optimize_duration = zeros(size(optimize_start));

    for i_field = 1:numel(optimize_field_names)
        optimize_timing = vertcat(experiment_result.timing.(optimize_field_names(i_field)));
        optimize_start(:, :, i_field) = optimize_timing(1:2:end, i_step);
        optimize_duration(:, :, i_field) = optimize_timing(2:2:end, i_step);
    end

    % Create directed_coupling_sequential for each optimize permutation
    % Current number of levels: nnz of optimize field duration
    permutations = squeeze(optimize_duration(1, 1, :) ~= 0);
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
        [~, latin_square] = sort(squeeze(optimize_start(:, 1, permutations)), 2);

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

            levels_per_vehicle(:, i_perm) = level_matrix * (1:size(latin_square, 2))';
        end

    end

    field_names = [ ...
                       "measure", ...
                       "analyze_reachability", ...
                       "receive_from_others", ...
                       "couple", ...
                       "prioritize", ...
                       "group", ...
                       optimize_field_names, ...
                       "receive_fallback", ...
                   ];

    % Find time of HLC which starts loop last
    measure_timings = vertcat(experiment_result.timing.measure);
    measure_start_points = measure_timings(1:2:end, :);
    t0 = max(measure_start_points(:, k));

    groups = conncomp(digraph(directed_coupling_sequential(:, :, 1)), Type = 'weak');

    [n_members, idx_group] = groupcounts(groups');

    [~, groups_by_members] = sort(n_members, "descend");

    group_order = idx_group(groups_by_members);

    tl = tiledlayout(options.amount, 2 * n_permutations, TileSpacing = "tight", Padding = "tight");

    maximum_time = 0;

    export_fig_config = ExportFigConfig.paper();

    for i_group = group_order'

        vehicle_list = 1:options.amount;
        vehicle_ids_in_group = vehicle_list(groups == i_group);

        nexttile([numel(vehicle_ids_in_group) n_permutations]);

        for field_i = 1:length(field_names)
            field_name = field_names(field_i);
            time_to_draw = nan(2, numel(vehicle_ids_in_group));

            for i_vehicle = 1:numel(vehicle_ids_in_group)

                veh_i = vehicle_ids_in_group(i_vehicle);
                timings = experiment_result.timing(veh_i);

                t_start = timings.(field_name)(1, k) - t0; % Normalize (see above)
                duration = timings.(field_name)(2, k);

                [~, id_order] = sort(levels_per_vehicle(vehicle_ids_in_group, 1));

                time_to_draw(:, id_order == i_vehicle) = ...
                    [t_start, t_start + duration] * 10^3; % Scale to ms
            end

            plot(time_to_draw, [1:numel(vehicle_ids_in_group); 1:numel(vehicle_ids_in_group)], 'SeriesIndex', field_i, ...
                LineWidth = 5, Tag = 'box_as_line');

            hold on;

        end

        maximum_time = max([time_to_draw(2, :), maximum_time]);

        yticks(1:numel(vehicle_ids_in_group));
        yticklabels(vehicle_ids_in_group(id_order))

        ylim([1 - 0.5, numel(vehicle_ids_in_group) + 0.5]);

        color_order = rwth_color_order();

        for i_permutation = 1:n_permutations

            n_colors = size(color_order, 1);
            i_color = mod(6 + i_permutation - 1, n_colors) + 1;

            nexttile([numel(vehicle_ids_in_group) 1]);
            sg = plot( ...
                digraph( ...
                directed_coupling_sequential( ...
                vehicle_list(groups == i_group), ...
                vehicle_list(groups == i_group), ...
                i_permutation ...
            ) ...
            ), ...
                NodeLabel = vehicle_list(vehicle_list(groups == i_group)), ...
                NodeColor = color_order(i_color, :), ...
                EdgeColor = color_order(i_color, :), ...
                MarkerSize = 2, ...
                ArrowSize = 4, ...
                NodeFontSize = export_fig_config.fontsize, ...
                NodeFontName = export_fig_config.fontname ...
            );
            sg.YData = levels_per_vehicle(vehicle_list(groups == i_group), i_permutation)';
            set(gca, 'YDir', 'reverse');
        end

    end

    % legend(plot_handle(1, :), strrep(cellstr(field_names), '_', ' '), Location = "bestoutside");
    % xlabel('Time [ms]');
    % xlim([-5, ceil(max(time_to_draw(2, :)))] + 5);
    % ylabel('Vehicle ID');

    % ylim([1 - 0.2, options.amount + 0.2]);
    for ax = tl.Children(1 + n_permutations:1 + n_permutations:end)'
        ax.XLim = [0, 5 + ceil(maximum_time)];
        ax.XGrid = 'on';

        if ax ~= tl.Children(n_permutations + 1)
            ax.XTickLabel = [];
        end

        ax.YDir = 'reverse';
    end

    % set_figure_properties(optional.fig, ExportFigConfig.document());

    if ~hold_before
        hold off;
    end

end
