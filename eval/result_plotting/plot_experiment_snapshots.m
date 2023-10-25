function plot_experiment_snapshots(results, step_indices, options)
    % OVERVIEWPLOT  Export plot with multiple snapshots.
    arguments
        results (1, 1) struct;
        step_indices (1, :) double
        options.do_export (1, 1) logical = false;
        options.fig (1, 1) matlab.ui.Figure = figure("Visible", "on");
        options.colorOffset (1, 1) double = 0
    end

    colors = rwth_color_order();

    scenario = results.scenario;

    nVeh = scenario.options.amount;
    nObst = size(results.obstacles, 1);

    nFigs = numel(step_indices);

    if nargin < 3
        tiledlayout(options.fig, nFigs, 1);
    end

    % show predictions for multiple timesteps
    for step = 1:nFigs
        step_idx = step_indices(step);

        nexttile(step)
        hold on
        box on

        if nargin < 3

            if step == numel(step_indices)
                xlabel('$x$ [m]', 'Interpreter', 'LaTex')
            end

            ylabel('$y$ [m]', 'Interpreter', 'LaTex')

            title(['Step ' num2str(step_idx)] ...
                , 'Interpreter', 'LaTex' ...
                , 'Units', 'normalized' ...
                , 'Position', [1.07, 0.5, 0] ...
                , 'VerticalAlignment', 'middle' ...
                , 'Rotation', 70 ...
            );

            % Obstacle rectangle
            for obs = 1:nObst
                patch(results.obstacles{obs}(1, :) ...
                    , results.obstacles{obs}(2, :) ...
                    , [0.5 0.5 0.5] ...
                );
            end

            % TODO plot dynamic obstacles from results

        end

        % past trajectory
        for v = 1:nVeh

            for iStep = 1:step_idx
                line(results.vehicle_path_fullres{v, iStep}(:, 1), ...
                    results.vehicle_path_fullres{v, iStep}(:, 2), ...
                    'Color', colors(v + options.colorOffset, :) ...
                );
            end

        end

        % predicted trajectory
        for v = 1:nVeh
            line(results.trajectory_predictions{v, step_idx}([1:scenario.options.tick_per_step + 1:end, end], 1), ...
                results.trajectory_predictions{v, step_idx}([1:scenario.options.tick_per_step + 1:end, end], 2), ...
                'Color', colors(v + options.colorOffset, :), 'LineStyle', 'none', 'Marker', 'o', 'MarkerFaceColor', colors(v + options.colorOffset, :), 'MarkerSize', 1);
            line(results.trajectory_predictions{v, step_idx}(:, 1), ...
                results.trajectory_predictions{v, step_idx}(:, 2), ...
                'Color', colors(v + options.colorOffset, :));
        end

        % Vehicle rectangles
        for v = 1:nVeh
            veh = scenario.vehicles(v);
            pos_step = results.vehicle_path_fullres{v, step_idx};
            x = pos_step(1, :);
            vehiclePolygon = transformed_rectangle(x(1), x(2), x(3), veh.Length, veh.Width);
            patch(vehiclePolygon(1, :) ...
                , vehiclePolygon(2, :) ...
                , colors(v + options.colorOffset, :) ...
            );
        end

        daspect([1 1 1]);
        xlim(scenario.options.plot_limits(1, :));
        ylim(scenario.options.plot_limits(2, :));

        if step == numel(step_indices)
            xlabel('$x$ [m]', 'Interpreter', 'LaTex')
        end

        ylabel('$y$ [m]', 'Interpreter', 'LaTex')

        title(['Step ' num2str(step_idx)], 'Interpreter', 'LaTex');
    end

    % lanelets
    if ~isempty(results.scenario.road_raw_data) && ~isempty(results.scenario.road_raw_data.lanelet)
        plot_lanelets(results.scenario.road_raw_data.lanelet, results.scenario.options.scenario_type);
    end

    set(options.fig.Children.Children(1), 'Units', 'centimeters');
    tile_height = options.fig.Children.Children(end).OuterPosition(4);
    x_axis_label_height = options.fig.Children.Children(1).OuterPosition(4) - tile_height;
    set(options.fig.Children.Children(1), 'Units', 'normalized');
    options.fig.Children.TileSpacing = 'compact';
    set_figure_properties(options.fig, ExportFigConfig.paper('paperheight', nFigs * (tile_height + x_axis_label_height)));

    if options.do_export
        results_folder = FileNameConstructor.gen_results_folder_path(results.scenario.options);
        filepath = fullfile(results_folder, 'snapshots.pdf');
        export_fig(options.fig, filepath)
        if (~options.fig.Visible); close(options.fig); end
    end

end
