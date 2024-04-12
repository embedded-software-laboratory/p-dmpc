function eval_coloring_paper()
    %% intro
    fprintf("\nEvaluation script for \'Reducing Computation Time with Priority Assignment in Distributed Control\'\n\n")

    foldername = 'coloring';
    folderpath = fullfile(FileNameConstructor.all_results(), foldername);
    mkdir(folderpath);

    %%
    % ███████╗██╗  ██╗██████╗ ███████╗██████╗ ██╗███╗   ███╗███████╗███╗   ██╗████████╗
    % ██╔════╝╚██╗██╔╝██╔══██╗██╔════╝██╔══██╗██║████╗ ████║██╔════╝████╗  ██║╚══██╔══╝
    % █████╗   ╚███╔╝ ██████╔╝█████╗  ██████╔╝██║██╔████╔██║█████╗  ██╔██╗ ██║   ██║
    % ██╔══╝   ██╔██╗ ██╔═══╝ ██╔══╝  ██╔══██╗██║██║╚██╔╝██║██╔══╝  ██║╚██╗██║   ██║
    % ███████╗██╔╝ ██╗██║     ███████╗██║  ██║██║██║ ╚═╝ ██║███████╗██║ ╚████║   ██║
    % ╚══════╝╚═╝  ╚═╝╚═╝     ╚══════╝╚═╝  ╚═╝╚═╝╚═╝     ╚═╝╚══════╝╚═╝  ╚═══╝   ╚═╝

    % Intersection experiment, 8 vehicles
    % 4 straight from left lane
    % 4 right from right lane
    config = Config.load_from_file("eval\eval_coloring\Config_coloring.json");
    copyfile("eval\eval_coloring\mcts.json", "Config\mcts.json");

    priority_strategies = [
                           PriorityStrategies.constant_priority
                           PriorityStrategies.random_priority
                           PriorityStrategies.FCA_priority
                           PriorityStrategies.coloring_priority
                           ];

    priority_names = [
                      "$p_{\mathrm{constant}}$"
                      "$p_{\mathrm{random}}$"
                      "$p_{\mathrm{constraint}}$"
                      "$p_{\mathrm{color}}$"
                      ];

    for i_priority_strategy = 1:numel(priority_strategies)
        config.priority = priority_strategies(i_priority_strategy);

        experiment_result = FileNameConstructor.load_latest(config);

        if isempty(experiment_result)
            experiment_result = main(config);
        end

        file_path = char(FileNameConstructor.experiment_result_folder_path(config));
        result_str_pos = strfind(file_path, 'results') + numel('results');
        file_path = [ ...
                         folderpath, filesep, ...
                         file_path(result_str_pos + 1:end), filesep, ...
                         '999999-999999.mat' ...
                     ];
        mkdir(fileparts(file_path));
        save(file_path, "experiment_result");

        experiment_results(i_priority_strategy) = experiment_result; %#ok<AGROW>
    end

    experiment_results = reshape(experiment_results, 1, [], 1);

    %%
    %  ██████╗ ██████╗ ███████╗████████╗
    % ██╔════╝██╔═══██╗██╔════╝╚══██╔══╝
    % ██║     ██║   ██║███████╗   ██║
    % ██║     ██║   ██║╚════██║   ██║
    % ╚██████╗╚██████╔╝███████║   ██║
    %  ╚═════╝ ╚═════╝ ╚══════╝   ╚═╝

    cost_percent_average = data_cost_percent(experiment_results);
    fig = figure;
    bar(priority_names, cost_percent_average);
    h_xaxis = get(gca, 'XAxis');
    h_xaxis.TickLabelInterpreter = 'latex';
    % axes
    xlabel("Prioritization algorithm")
    ylabel( ...
        "$J_\mathrm{NCS}(p) / J_\mathrm{NCS}(p_\mathrm{const})$ [\%]", ...
        Interpreter = "latex" ...
    );

    set_figure_properties(fig, ExportFigConfig.paper());
    filename = sprintf('prioritization_cost.pdf');
    filepath = fullfile(folderpath, filename);
    export_fig(fig, filepath);
    close all;

    %%
    % ████████╗██╗███╗   ███╗███████╗
    % ╚══██╔══╝██║████╗ ████║██╔════╝
    %    ██║   ██║██╔████╔██║█████╗
    %    ██║   ██║██║╚██╔╝██║██╔══╝
    %    ██║   ██║██║ ╚═╝ ██║███████╗
    %    ╚═╝   ╚═╝╚═╝     ╚═╝╚══════╝
    [~, time_med_approach_vehicle, ~, time_max_approach_vehicle] = data_time_approach_vehicle( ...
        experiment_results, ...
        computation_time_function = @data_time_prioritize_optimize_experiment ...
    );

    fig = figure;
    max_bar = bar(priority_names, time_max_approach_vehicle' .* 1000);
    hold on
    med_bar = bar(priority_names, time_med_approach_vehicle' .* 1000);

    h_xaxis = get(gca, 'XAxis');
    h_xaxis.TickLabelInterpreter = 'latex';
    % legend
    legendtext = ["med", "max"];
    legend([med_bar, max_bar], legendtext, Location = 'best', Interpreter = 'latex');
    % axes
    xlabel("Prioritization algorithm")
    ylabel('$T_{\mathrm{NCS}}$ [ms]', Interpreter = 'latex');

    set_figure_properties(fig, ExportFigConfig.paper());
    rwth_colors_100 = rwth_color_order();
    rwth_colors_50 = rwth_color_order_50;
    colororder( ...
        fig, ...
        [rwth_colors_50(1, :); ...
         rwth_colors_100(1, :)] ...
    );

    filename = sprintf('prioritization_time.pdf');
    filepath = fullfile(folderpath, filename);
    export_fig(fig, filepath);
    close all;

    %%
    % ████████╗██╗███╗   ███╗███████╗    ███╗   ██╗ ██████╗ ██████╗ ███╗   ███╗
    % ╚══██╔══╝██║████╗ ████║██╔════╝    ████╗  ██║██╔═══██╗██╔══██╗████╗ ████║
    %    ██║   ██║██╔████╔██║█████╗      ██╔██╗ ██║██║   ██║██████╔╝██╔████╔██║
    %    ██║   ██║██║╚██╔╝██║██╔══╝      ██║╚██╗██║██║   ██║██╔══██╗██║╚██╔╝██║
    %    ██║   ██║██║ ╚═╝ ██║███████╗    ██║ ╚████║╚██████╔╝██║  ██║██║ ╚═╝ ██║██╗
    %    ╚═╝   ╚═╝╚═╝     ╚═╝╚══════╝    ╚═╝  ╚═══╝ ╚═════╝ ╚═╝  ╚═╝╚═╝     ╚═╝╚═╝
    max_time = max(time_max_approach_vehicle, [], "all");
    time_max_approach_vehicle_normalized = time_max_approach_vehicle ./ max_time;
    time_med_approach_vehicle_normalized = time_med_approach_vehicle ./ max_time;

    fig = figure;
    max_bar = bar(priority_names, time_max_approach_vehicle_normalized' .* 100);
    hold on
    med_bar = bar(priority_names, time_med_approach_vehicle_normalized' .* 100);

    h_xaxis = get(gca, 'XAxis');
    h_xaxis.TickLabelInterpreter = 'latex';
    % legend
    legendtext = ["med", "max"];
    legend([med_bar, max_bar], legendtext, Location = 'best', Interpreter = 'latex');
    % axes
    xlabel("Prioritization algorithm")
    ylabel('$T_{\mathrm{NCS}} / T_{\mathrm{NCS, max}}$ [\%]', Interpreter = 'latex');

    set_figure_properties(fig, ExportFigConfig.paper());
    rwth_colors_100 = rwth_color_order;
    rwth_colors_50 = rwth_color_order_50;
    colororder( ...
        fig, ...
        [rwth_colors_50(1, :); ...
         rwth_colors_100(1, :)] ...
    );

    filename = sprintf('prioritization_time_normalized.pdf');
    filepath = fullfile(folderpath, filename);
    export_fig(fig, filepath);
    close all;

    %%
    % ████████╗██╗███╗   ███╗███████╗    ██████╗ ██████╗ ██╗ ██████╗
    % ╚══██╔══╝██║████╗ ████║██╔════╝    ██╔══██╗██╔══██╗██║██╔═══██╗
    %    ██║   ██║██╔████╔██║█████╗      ██████╔╝██████╔╝██║██║   ██║
    %    ██║   ██║██║╚██╔╝██║██╔══╝      ██╔═══╝ ██╔══██╗██║██║   ██║
    %    ██║   ██║██║ ╚═╝ ██║███████╗    ██║     ██║  ██║██║╚██████╔╝
    %    ╚═╝   ╚═╝╚═╝     ╚═╝╚══════╝    ╚═╝     ╚═╝  ╚═╝╚═╝ ╚═════╝

    filename = 'prioritization_time_coloring.txt';
    filepath = fullfile(folderpath, filename);
    fileID = fopen(filepath, 'w');

    for experiment_result = experiment_results
        prioritize_timing = vertcat(experiment_result.timing.prioritize);
        prioritize_duration = max(prioritize_timing(2:2:end, :), [], 1);
        prioritize_duration_max = max(prioritize_duration) * 1000;
        prioritize_duration_med = median(prioritize_duration) * 1000;
        str_to_write = sprintf( ...
            "Prioritization time for %17s -- max: %5.2f ms -- med: %5.2f ms\n" ...
            , experiment_result.options.priority ...
            , prioritize_duration_max ...
            , prioritize_duration_med ...
        );
        fwrite(fileID, str_to_write);
    end

    fclose(fileID);

    %%
    % ██╗     ███████╗██╗   ██╗███████╗██╗     ███████╗
    % ██║     ██╔════╝██║   ██║██╔════╝██║     ██╔════╝
    % ██║     █████╗  ██║   ██║█████╗  ██║     ███████╗
    % ██║     ██╔══╝  ╚██╗ ██╔╝██╔══╝  ██║     ╚════██║
    % ███████╗███████╗ ╚████╔╝ ███████╗███████╗███████║
    % ╚══════╝╚══════╝  ╚═══╝  ╚══════╝╚══════╝╚══════╝

    [~, n_levels_med_approach_vehicle, ~, n_levels_max_approach_vehicle] = data_n_levels_approach_vehicle(experiment_results);

    fig = figure;
    max_bar = bar(priority_names, n_levels_max_approach_vehicle');
    hold on
    med_bar = bar(priority_names, n_levels_med_approach_vehicle');

    h_xaxis = get(gca, 'XAxis');
    h_xaxis.TickLabelInterpreter = 'latex';
    % legend
    legend([med_bar, max_bar], legendtext, Location = 'best', Interpreter = 'latex');
    % axes
    xlabel("Prioritization algorithm")
    ylabel('$N_{\mathrm{CL}}$', Interpreter = 'latex');

    set_figure_properties(fig, ExportFigConfig.paper());
    rwth_colors_100 = rwth_color_order;
    rwth_colors_50 = rwth_color_order_50;
    colororder( ...
        fig, ...
        [rwth_colors_50(1, :); ...
         rwth_colors_100(1, :)] ...
    );

    filename = sprintf('prioritization_levels.pdf');
    filepath = fullfile(folderpath, filename);
    export_fig(fig, filepath);
    close all;

    %%
    % ██╗     ███████╗██╗   ██╗███████╗██╗         ██████╗ ██╗███████╗████████╗
    % ██║     ██╔════╝██║   ██║██╔════╝██║         ██╔══██╗██║██╔════╝╚══██╔══╝
    % ██║     █████╗  ██║   ██║█████╗  ██║         ██║  ██║██║███████╗   ██║
    % ██║     ██╔══╝  ╚██╗ ██╔╝██╔══╝  ██║         ██║  ██║██║╚════██║   ██║
    % ███████╗███████╗ ╚████╔╝ ███████╗███████╗    ██████╔╝██║███████║   ██║
    % ╚══════╝╚══════╝  ╚═══╝  ╚══════╝╚══════╝    ╚═════╝ ╚═╝╚══════╝   ╚═╝
    % levels per acyclic orientation
    lvl_dist = calc_level_dist(experiment_results(1).iteration_data(1).adjacency);

    fig = figure();
    levels = sort(unique(lvl_dist));
    bin_margin = 0.3;
    bin_edges_low = levels - bin_margin;
    bin_edges_high = levels + bin_margin;
    bin_edges = sort([bin_edges_low bin_edges_high]);
    histogram( ...
        lvl_dist ...
        , FaceAlpha = 1 ...
        , Orientation = 'horizontal' ...
        , BinEdges = bin_edges ...
    );
    set(gca, 'xscale', 'log');
    set(gca, 'YTick', 1:size(experiment_results(1).iteration_data(1).adjacency, 1));
    xlabel('Number of acyclic orientations', Interpreter = 'LaTex');
    ylabel('$N_{\mathrm{CL}}$', Interpreter = 'LaTex');
    set_figure_properties(fig, ExportFigConfig.paper());
    export_fig(fig, fullfile(folderpath, 'level_distribution.pdf'));
    close(fig);

    %%
    % ███████╗ ██████╗███████╗███╗   ██╗ █████╗ ██████╗ ██╗ ██████╗
    % ██╔════╝██╔════╝██╔════╝████╗  ██║██╔══██╗██╔══██╗██║██╔═══██╗
    % ███████╗██║     █████╗  ██╔██╗ ██║███████║██████╔╝██║██║   ██║
    % ╚════██║██║     ██╔══╝  ██║╚██╗██║██╔══██║██╔══██╗██║██║   ██║
    % ███████║╚██████╗███████╗██║ ╚████║██║  ██║██║  ██║██║╚██████╔╝
    % ╚══════╝ ╚═════╝╚══════╝╚═╝  ╚═══╝╚═╝  ╚═╝╚═╝  ╚═╝╚═╝ ╚═════╝
    experiment_result = experiment_results(1);
    time_step = 1;
    vehicle_indices = 1:experiment_result.options.amount;
    plotting_info = PlottingInfo(vehicle_indices, experiment_result, time_step);
    fig = figure;
    grey = [0.5 0.5 0.5];
    black = [0 0 0];
    scenario = Scenario.create(experiment_result.options);
    vehicles = scenario.vehicles;
    export_fig_config = ExportFigConfig.paper( ...
        paperheight = 8 ...
        , markersize = 1 ...
    );

    axis equal
    daspect([1 1 1])
    xlabel('$x$ [m]', Interpreter = 'LaTex');
    ylabel('$y$ [m]', Interpreter = 'LaTex');
    xlim([1.25 3.25]);
    ylim([1.0 3.0]);
    set_figure_properties(fig, export_fig_config);
    %
    t = text( ...
        0.5, 0.5, ...
        '20', ...
        LineWidth = 1, ...
        Color = 'black', ...
        HorizontalAlignment = 'center', ...
        FontSize = export_fig_config.fontsize, ...
        Tag = "temporary" ...
    );
    extent = t.Extent;
    delete(t);
    text_width = extent(3);
    text_height = extent(4);
    % compute new radius so that text fits into circle
    vehicle_id_radius = max(text_width, text_height) * 1.2/2;

    plot_lanelets(scenario.road_raw_data.lanelet, color = [0.8 0.8 0.8]);

    for i_vehicle = vehicle_indices
        % Sampled reference trajectory points
        line( ...
            plotting_info.ref_trajectory(i_vehicle, :, 1), ...
            plotting_info.ref_trajectory(i_vehicle, :, 2), ...
            Color = 'black', ...
            LineStyle = 'none', ...
            Marker = 'o', ...
            MarkerFaceColor = black, ...
            MarkerSize = 0.5, ...
            Tag = "temporary" ...
        );

        % Vehicle rectangles
        x = plotting_info.x0(:, i_vehicle);
        veh = vehicles(i_vehicle);
        vehiclePolygon = transformed_rectangle(x(1), x(2), x(3), veh.Length, veh.Width);
        patch( ...
            vehiclePolygon(1, :) ...
            , vehiclePolygon(2, :) ...
            , grey ...
            , EdgeColor = 'none' ...
            , Tag = "temporary" ...
        );

        % plot the vehicle ID in the middle of each vehicle on a lighter background
        rectangle( ...
            Position = [x(1) - vehicle_id_radius, x(2) - vehicle_id_radius, 2 * vehicle_id_radius, 2 * vehicle_id_radius], ...
            Curvature = [1, 1], ...
            FaceColor = [1, 1, 1, 1], ...
            LineStyle = '-', ...
            LineWidth = 0.5, ...
            Tag = 'circle' ...
        );

        text( ...
            x(1), x(2), ...
            num2str(i_vehicle), ...
            LineWidth = 0.5, ...
            Color = 'black', ...
            HorizontalAlignment = 'center', ...
            FontSize = export_fig_config.fontsize, ...
            Tag = "temporary" ...
        );

    end

    % plot coupling
    x0 = plotting_info.x0';

    directed_coupling = experiment_result.iteration_data(time_step).directed_coupling;

    for v = 1:experiment_result.options.amount
        x_base = x0(v, :);
        adjacent_vehicles = find(directed_coupling(v, :));

        for adj_v = adjacent_vehicles
            adj_x = x0(adj_v, :);

            % plot adjacency
            % Only use position.

            u = (adj_x - x_base)';
            x = x_base(1:2)';
            u = u(1:2);

            % Transform arrow size so that it starts and ends not in the vehicle middle, but at a circle around the center with given radius.
            l = sqrt(u(1) * u(1) + u(2) * u(2));
            x = x + vehicle_id_radius * u / l;
            u = u - 2 * vehicle_id_radius * u / l;

            % Plot.
            line( ...
                [x(1), x(1) + u(1)], ...
                [x(2), x(2) + u(2)], ...
                Color = black, ...
                LineStyle = '-', ...
                LineWidth = 0.5, ...
                Tag = "temporary" ...
            );
        end

    end

    set_figure_properties(fig, export_fig_config);
    export_fig(fig, fullfile(folderpath, 'scenario.pdf'));
    close(fig);

    %%
    % ██████╗  █████╗  ██████╗
    % ██╔══██╗██╔══██╗██╔════╝
    % ██║  ██║███████║██║  ███╗
    % ██║  ██║██╔══██║██║   ██║
    % ██████╔╝██║  ██║╚██████╔╝
    % ╚═════╝ ╚═╝  ╚═╝ ╚═════╝

    common_steps = true(1, experiment_results(1).n_steps);

    for experiment_result = experiment_results
        levels_over_time = [experiment_result.iteration_data.number_of_computation_levels];
        max_level = max(levels_over_time);
        steps_with_max_level = levels_over_time == max_level;
        common_steps = common_steps & steps_with_max_level;
    end

    step = find(common_steps, 1);

    if isempty(step)
        warning("no common step with maximum levels, taking the first step")
        step = 1;
    end

    fig = figure;
    i_experiments = [1 4];
    n_figure_cols = 2;
    n_figure_rows = ceil(numel(i_experiments) / n_figure_cols);
    tiledlayout(fig, n_figure_rows, n_figure_cols);

    % show predictions for multiple timesteps
    for i_fig = i_experiments

        nexttile
        hold on
        box on

        p = plot( ...
            digraph(experiment_results(i_fig).iteration_data(step).directed_coupling), ...
            ArrowSize = 5, ...
            MarkerSize = 3, ...
            NodeColor = black, ...
            EdgeColor = grey, ...
            EdgeAlpha = 1, ...
            ArrowPosition = 0.95 ...
        );

        ylim([0.5 8.5]);

        if i_fig == 4 % coloring
            p.NodeColor = rwth_colors_100(p.YData, :);
            ylim([-4.5 3.5]);
        end

        title(priority_names(i_fig), Interpreter = 'latex');

        layout(p, 'layered', AssignLayers = 'asap');

        if mod(i_fig - 1, n_figure_cols) == 0
            ylabel('Computation level', Interpreter = 'LaTex')
            yticklabels(string(8:-1:1))
        else
            set(gca, 'yticklabel', [])
        end

        set(gca, 'xticklabel', [])
    end

    fig.Children.TileSpacing = 'compact';
    fig.Children.Padding = 'compact';

    set_figure_properties(fig, ExportFigConfig.paper(paperheight = 6));
    export_fig(fig, fullfile(folderpath, 'dag.pdf'));
    close(fig);

end
