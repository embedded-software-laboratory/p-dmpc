function hdv_reachable_set_plot(result, step_idx, tick_now, visu)

    arguments
        result
        step_idx = 1;
        tick_now = 1;
        visu OptionsPlotOnline = OptionsPlotOnline();
    end

    rwth_color = rwth_color_order();
    x_plot_limits = [1, 3.5];
    y_plot_limits = [3.2, 4.4];

    close all

    scenario = result.scenario;
    iter = result.iteration_structs{step_idx};

    nVeh = scenario.options.amount;

    %% 1 complete reachable sets
    fig1 = figure();

    hold on
    axis equal

    xlabel('$x$ [m]', 'Interpreter', 'LaTex');
    ylabel('$y$ [m]', 'Interpreter', 'LaTex');

    xlim(x_plot_limits);
    ylim(y_plot_limits);

    % plot lanelets at the beginning
    if ~isempty(scenario.road_raw_data)

        if ~isempty(scenario.road_raw_data.lanelet)
            plot_lanelets(scenario.road_raw_data.lanelet, scenario.options.scenario_name);
        end

    end

    % plot sets backwards for better visability
    for i = size(iter.reachable_sets, 2):-1:1
        plot(iter.reachable_sets{1, i}, 'FaceColor', rwth_color(3, :), 'FaceAlpha', 0.1)
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

    % export as pdf
    set_figure_properties(fig1, ExportFigConfig.paper)
    filepath = fullfile('results', 'hdv_reachable_set.pdf');
    export_fig(fig1, filepath);
    close(fig1);

    %% 2 reachable sets instersected with lanelets

    fig2 = figure();

    axis equal
    hold on

    xlabel('$x$ [m]', 'Interpreter', 'LaTex');
    ylabel('$y$ [m]', 'Interpreter', 'LaTex');

    xlim(x_plot_limits);
    ylim(y_plot_limits);

    % plot the lanelets at the beginning
    if ~isempty(scenario.road_raw_data)

        if ~isempty(scenario.road_raw_data.lanelet)
            plot_lanelets(scenario.road_raw_data.lanelet, scenario.options.scenario_name);
        end

    end

    % create one polyshape containing all lanelets
    all_lanelet_boundary = [scenario.lanelet_boundary{:}];
    all_lanes_poly = union([all_lanelet_boundary{3:3:end}], 'KeepCollinearPoints', false);

    % plot sets backwards for better visability
    for i = size(iter.reachable_sets, 2):-1:1
        plot(intersect(iter.reachable_sets{1, i}, all_lanes_poly), 'FaceColor', rwth_color(3, :), 'FaceAlpha', 0.1)
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

    % export as pdf
    set_figure_properties(fig2, ExportFigConfig.paper)
    filepath = fullfile('results', 'hdv_all_lanes.pdf');
    export_fig(fig2, filepath);
    close(fig2);

    %% 3 reachable set considering RSS

    fig3 = figure();

    hold on
    axis equal

    xlabel('$x$ [m]', 'Interpreter', 'LaTex');
    ylabel('$y$ [m]', 'Interpreter', 'LaTex');

    xlim(x_plot_limits);
    ylim(y_plot_limits);

    % plot the lanelets at the beginning
    if ~isempty(scenario.road_raw_data)

        if ~isempty(scenario.road_raw_data.lanelet)
            plot_lanelets(scenario.road_raw_data.lanelet, scenario.options.scenario_name);
        end

    end

    % create polyshape containing current, predecessor and successor lanelet
    pos_step = result.trajectory_predictions{1, step_idx};
    x = pos_step(tick_now, :);
    lanelet_struct = scenario.road_raw_data.lanelet;

    current_lanelet_id = match_pose_to_lane(scenario, x(1), x(2), 1);
    successor_lanelet_id = lanelet_struct(current_lanelet_id).successor.refAttribute;
    predecessor_lanelet_id = lanelet_struct(current_lanelet_id).predecessor.refAttribute;
    current_lanelet_poly = polyshape([lanelet_struct(current_lanelet_id).leftBound.point(:).x, lanelet_struct(current_lanelet_id).rightBound.point(end:-1:1).x
        lanelet_struct(current_lanelet_id).leftBound.point(:).y, lanelet_struct(current_lanelet_id).rightBound.point(end:-1:1).y]');
    successor_lanelet_poly = polyshape([lanelet_struct(successor_lanelet_id).leftBound.point(:).x, lanelet_struct(successor_lanelet_id).rightBound.point(end:-1:1).x
        lanelet_struct(successor_lanelet_id).leftBound.point(:).y, lanelet_struct(successor_lanelet_id).rightBound.point(end:-1:1).y]');
    predecessor_lanelet_poly = polyshape([lanelet_struct(predecessor_lanelet_id).leftBound.point(:).x, lanelet_struct(predecessor_lanelet_id).rightBound.point(end:-1:1).x
        lanelet_struct(predecessor_lanelet_id).leftBound.point(:).y, lanelet_struct(predecessor_lanelet_id).rightBound.point(end:-1:1).y]');
    lanelets_poly = union([current_lanelet_poly, successor_lanelet_poly, predecessor_lanelet_poly]);

    % plot sets backwards for better visability
    for i = size(iter.reachable_sets, 2):-1:1
        plot(intersect(iter.reachable_sets{1, i}, lanelets_poly), 'FaceColor', rwth_color(3, :), 'FaceAlpha', 0.1)
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

    % export as pdf
    set_figure_properties(fig3, ExportFigConfig.paper)
    filepath = fullfile('results', 'hdv_own_lane.pdf');
    export_fig(fig3, filepath);
    close(fig3);

end
