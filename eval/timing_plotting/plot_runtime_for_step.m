function plot_runtime_for_step(result, k)
    %% PLOT_RUNTIME_FOR_STEP plots the timing of the specified timestep

    arguments
        result struct;
        k uint64;
    end

    % Configure, which field names in the timing object are relevant, dependent on the used controller
    if result.scenario.options.is_prioritized & result.scenario.options.compute_in_parallel
        field_names = [ ...
                           "plan_single_vehicle", ...
                           "optimizer", ...
                           "fallback", ...
                           "publish_predictions" ...
                       ];
        timings = result.timings_per_vehicle; % Will be unnecessary as soon as only one timing object exists
    else
        error('The graph is currently only supported for results of prioritized, distributed execution.');
    end

    % axes to plot in
    ax = axes;

    categories = categorical(field_names);
    disp(categories);

    for i = 1:length(field_names)
        field_name = field_names(i);
        disp(field_name);
        t_start(i) = timings.(field_name)(1, 1, k);
        duration = timings.(field_name)(2, 1, k);
        t_end(i) = t_start(i) + duration;

        % disp([c; c]);
        % disp([t_start; t_end]);
        % plot([t_start, t_start + 1; t_end, t_end + 1], [c; c], 'LineWidth', 5);

        % break;

        %     barh(i, t_end);
        %     hold on;
        %     bh1 = barh(i, t_start);
        %     bh1.FaceColor = [1 1 1];
        %     bh1.LineStyle = 'None';
        %     bh1.EdgeColor = [1 1 1];
    end

    disp(t_start);
    disp(t_end);

    plot([t_start; t_end], [categories; categories], 'LineWidth', 5);

    set(gca, 'TickLabelInterpreter', 'none');

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % minimum = rand(10, 3);
    % maximum = rand(10, 3) + minimum;

    % bH = bar(maximum);
    % hold on;
    % bH1 = bar(minimum);

    % for ii = bH1
    %     ii.FaceColor = [1 1 1];
    %     ii.LineStyle = 'None';
    %     ii.EdgeColor = [1 1 1];
    % end

    % for ii = bH
    %     ii.LineStyle = 'None';
    %     ii.EdgeColor = [1 1 1];
    % end

    % axes('Position', aH.Position, 'XTick', [], 'YTick', [], 'Color', 'None');

end
