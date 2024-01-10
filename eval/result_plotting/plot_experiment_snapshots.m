function plot_experiment_snapshots(experiment_result, step_indices, optional)
    % OVERVIEWPLOT  Export plot with multiple snapshots.
    arguments
        experiment_result (1, 1) ExperimentResult;
        step_indices (1, :) double
        optional.do_export (1, 1) logical = true;
        optional.fig (1, 1) matlab.ui.Figure = figure("Visible", "on");
    end

    options = experiment_result.options;
    scenario = experiment_result.scenario;

    n_vehicles = options.amount;
    obstacles = experiment_result.iteration_data{experiment_result.n_steps}.obstacles;
    nObst = size(obstacles, 1);

    nFigs = numel(step_indices);

    if nargin < 3
        tiledlayout(optional.fig, nFigs, 1);
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
                patch(obstacles{obs}(1, :) ...
                    , obstacles{obs}(2, :) ...
                    , [0.5 0.5 0.5] ...
                );
            end

        end

        % past trajectory
        plot_trajectories( ...
            experiment_result, ...
            fig = optional.fig, ...
            do_export = false, ...
            time_span = [1, step_idx] ...
        );

        % predicted trajectory
        for i_vehicle = 1:n_vehicles
            line(experiment_result.trajectory_predictions{i_vehicle, step_idx}([1:options.tick_per_step + 1:end, end], 1), ...
                experiment_result.trajectory_predictions{i_vehicle, step_idx}([1:options.tick_per_step + 1:end, end], 2), ...
                'Color', rwth_color_order(i_vehicle), 'LineStyle', 'none', 'Marker', 'o', 'MarkerFaceColor', rwth_color_order(i_vehicle), 'MarkerSize', 1);
            line(experiment_result.trajectory_predictions{i_vehicle, step_idx}(:, 1), ...
                experiment_result.trajectory_predictions{i_vehicle, step_idx}(:, 2), ...
                'Color', rwth_color_order(i_vehicle));

            % Vehicle rectangles
            veh = scenario.vehicles(i_vehicle);
            state = experiment_result.iteration_data{step_idx}.x0(i_vehicle, :);
            vehiclePolygon = transformed_rectangle(state(1), state(2), state(3), veh.Length, veh.Width);
            patch(vehiclePolygon(1, :) ...
                , vehiclePolygon(2, :) ...
                , rwth_color_order(i_vehicle) ...
            );
        end

        % lanelets
        if ~isempty(scenario.road_raw_data) && ~isempty(scenario.road_raw_data.lanelet)
            plot_lanelets(scenario.road_raw_data.lanelet);
        end

        daspect([1 1 1]);
        xlim(options.plot_limits(1, :));
        ylim(options.plot_limits(2, :));

        if step == numel(step_indices)
            xlabel('$x$ [m]', 'Interpreter', 'LaTex')
        end

        ylabel('$y$ [m]', 'Interpreter', 'LaTex')

        title(['Step ' num2str(step_idx)], 'Interpreter', 'LaTex');
    end

    set(optional.fig.Children.Children(1), 'Units', 'centimeters');
    tile_height = optional.fig.Children.Children(end).OuterPosition(4);
    x_axis_label_height = optional.fig.Children.Children(1).OuterPosition(4) - tile_height;
    set(optional.fig.Children.Children(1), 'Units', 'normalized');
    optional.fig.Children.TileSpacing = 'compact';
    set_figure_properties(optional.fig, ExportFigConfig.paper('paperheight', nFigs * (tile_height + x_axis_label_height)));

    if optional.do_export
        results_folder = FileNameConstructor.gen_results_folder_path(options);
        filepath = fullfile(results_folder, 'snapshots.pdf');
        export_fig(optional.fig, filepath)
    end

    if (~optional.fig.Visible); close(optional.fig); end

end
