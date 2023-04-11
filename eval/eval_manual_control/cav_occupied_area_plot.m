function cav_occupied_area_plot(result, step_idx, tick_now, visu)

    arguments
        result
        step_idx = 1;
        tick_now = 1;
        visu OptionsPlotOnline = OptionsPlotOnline();
    end

    rwth_color = rwth_color_order();
    x_plot_limits = [1.0, 3.5];
    y_plot_limits = [3.2, 4.4];

    close all

    scenario = result.scenario;
    predictions = result.trajectory_predictions{1, step_idx};

    nVeh = scenario.options.amount;

    %% 1
    fig1 = figure();

    hold on
    axis equal

    xlabel('$x$ [m]', 'Interpreter', 'LaTex');
    ylabel('$y$ [m]', 'Interpreter', 'LaTex');

    xlim(x_plot_limits);
    ylim(y_plot_limits);

    % plot the lanelets only once at the beginning
    if ~isempty(scenario.road_raw_data)

        if ~isempty(scenario.road_raw_data.lanelet)
            plot_lanelets(scenario.road_raw_data.lanelet, scenario.options.scenario_name);
        end

    end

    n_predictions = size(predictions, 1);
    n_ticks = n_predictions / scenario.options.Hp;

    for tick = 1:n_ticks:(n_predictions - n_ticks)
        x = predictions(tick, :);
        trim1 = predictions(tick, 4);
        trim2 = predictions(tick + n_ticks, 4);
        area = scenario.mpa.maneuvers{trim1, trim2}.area;
        [area_x, area_y] = translate_global(x(3), x(1), x(2), area(1, :), area(2, :));
        area_poly = polyshape([area_x; area_y]');
        plot(area_poly, 'FaceColor', rwth_color(3, :), 'FaceAlpha', 0.1)
    end

    % Vehicle rectangles
    for v = 1:nVeh
        veh = scenario.vehicles(v);
        pos_step = result.trajectory_predictions{v, step_idx};
        x = pos_step(tick_now, :);
        vehiclePolygon = transformed_rectangle(x(1), x(2), x(3), veh.Length, veh.Width);
        patch(vehiclePolygon(1, :) ...
            , vehiclePolygon(2, :) ...
            , rwth_color(v, :) ...
            , 'LineWidth', 1 ...
        );

        if visu.plot_vehicle_id
            radius = veh.Width * 0.95/2;
            rectangle('Position', [x(1) - radius, x(2) - radius, 2 * radius, 2 * radius], 'Curvature', [1, 1], ...
                'FaceColor', [1, 1, 1, 0.75], 'LineStyle', 'none', 'LineWidth', 1, 'Tag', 'circle');
            text(x(1), x(2), num2str(v), 'FontSize', 5, 'LineWidth', 1, 'Color', 'black', 'HorizontalAlignment', 'center');
        end

    end

    set_figure_properties(fig1, ExportFigConfig.paper)
    filepath = fullfile('results', 'cav_occupied_area.pdf');
    export_fig(fig1, filepath);
    close(fig1);
end
