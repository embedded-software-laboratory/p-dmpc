function eval_coloring_paper()
    %% intro
    fprintf("\nEvaluation script for \'Reducing Computation Time with Priority Assignment in Distributed Control\'\n\n")

    % ███████╗██╗  ██╗██████╗ ███████╗██████╗ ██╗███╗   ███╗███████╗███╗   ██╗████████╗
    % ██╔════╝╚██╗██╔╝██╔══██╗██╔════╝██╔══██╗██║████╗ ████║██╔════╝████╗  ██║╚══██╔══╝
    % █████╗   ╚███╔╝ ██████╔╝█████╗  ██████╔╝██║██╔████╔██║█████╗  ██╔██╗ ██║   ██║
    % ██╔══╝   ██╔██╗ ██╔═══╝ ██╔══╝  ██╔══██╗██║██║╚██╔╝██║██╔══╝  ██║╚██╗██║   ██║
    % ███████╗██╔╝ ██╗██║     ███████╗██║  ██║██║██║ ╚═╝ ██║███████╗██║ ╚████║   ██║
    % ╚══════╝╚═╝  ╚═╝╚═╝     ╚══════╝╚═╝  ╚═╝╚═╝╚═╝     ╚═╝╚══════╝╚═╝  ╚═══╝   ╚═╝

    %% Intersection experiment, 8 vehicles
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

        experiment_results(i_priority_strategy) = experiment_result; %#ok<AGROW>
    end

    experiment_results = reshape(experiment_results, 1, [], 1);

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
    filename = sprintf('prioritization_cost_coloring.pdf');
    filepath = fullfile(FileNameConstructor.all_results(), filename);
    export_fig(fig, filepath);
    close all;

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
    rwth_colors_100 = rwth_color_order;
    rwth_colors_50 = rwth_color_order_50;
    colororder( ...
        fig, ...
        [rwth_colors_50(1, :); ...
         rwth_colors_100(1, :)] ...
    );

    filename = sprintf('prioritization_time_coloring.pdf');
    filepath = fullfile(FileNameConstructor.all_results(), filename);
    export_fig(fig, filepath);
    close all;

    % ████████╗██╗███╗   ███╗███████╗    ███╗   ██╗ ██████╗ ██████╗ ███╗   ███╗ █████╗ ██╗     ██╗███████╗███████╗██████╗
    % ╚══██╔══╝██║████╗ ████║██╔════╝    ████╗  ██║██╔═══██╗██╔══██╗████╗ ████║██╔══██╗██║     ██║╚══███╔╝██╔════╝██╔══██╗
    %    ██║   ██║██╔████╔██║█████╗      ██╔██╗ ██║██║   ██║██████╔╝██╔████╔██║███████║██║     ██║  ███╔╝ █████╗  ██║  ██║
    %    ██║   ██║██║╚██╔╝██║██╔══╝      ██║╚██╗██║██║   ██║██╔══██╗██║╚██╔╝██║██╔══██║██║     ██║ ███╔╝  ██╔══╝  ██║  ██║
    %    ██║   ██║██║ ╚═╝ ██║███████╗    ██║ ╚████║╚██████╔╝██║  ██║██║ ╚═╝ ██║██║  ██║███████╗██║███████╗███████╗██████╔╝
    %    ╚═╝   ╚═╝╚═╝     ╚═╝╚══════╝    ╚═╝  ╚═══╝ ╚═════╝ ╚═╝  ╚═╝╚═╝     ╚═╝╚═╝  ╚═╝╚══════╝╚═╝╚══════╝╚══════╝╚═════╝
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

    filename = sprintf('prioritization_time_normalized_coloring.pdf');
    filepath = fullfile(FileNameConstructor.all_results(), filename);
    export_fig(fig, filepath);
    close all;

    % ██╗     ███████╗██╗   ██╗███████╗██╗     ███████╗
    % ██║     ██╔════╝██║   ██║██╔════╝██║     ██╔════╝
    % ██║     █████╗  ██║   ██║█████╗  ██║     ███████╗
    % ██║     ██╔══╝  ╚██╗ ██╔╝██╔══╝  ██║     ╚════██║
    % ███████╗███████╗ ╚████╔╝ ███████╗███████╗███████║
    % ╚══════╝╚══════╝  ╚═══╝  ╚══════╝╚══════╝╚══════╝

    [~, time_med_approach_vehicle, ~, time_max_approach_vehicle] = data_n_levels_approach_vehicle(experiment_results);

    fig = figure;
    max_bar = bar(priority_names, time_max_approach_vehicle');
    hold on
    med_bar = bar(priority_names, time_med_approach_vehicle');

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

    filename = sprintf('prioritization_levels_coloring.pdf');
    filepath = fullfile(FileNameConstructor.all_results(), filename);
    export_fig(fig, filepath);
    close all;

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
    export_fig(fig, fullfile('./results/levels.pdf'));
    close(fig);

    % ███████╗ ██████╗███████╗███╗   ██╗ █████╗ ██████╗ ██╗ ██████╗
    % ██╔════╝██╔════╝██╔════╝████╗  ██║██╔══██╗██╔══██╗██║██╔═══██╗
    % ███████╗██║     █████╗  ██╔██╗ ██║███████║██████╔╝██║██║   ██║
    % ╚════██║██║     ██╔══╝  ██║╚██╗██║██╔══██║██╔══██╗██║██║   ██║
    % ███████║╚██████╗███████╗██║ ╚████║██║  ██║██║  ██║██║╚██████╔╝
    % ╚══════╝ ╚═════╝╚══════╝╚═╝  ╚═══╝╚═╝  ╚═╝╚═╝  ╚═╝╚═╝ ╚═════╝

    % TODO plot scenario with undirected coupling
    % see `plot_experiment_snapshots`
    %    - first time step
    %    - all vehicles same color
    %    - reference trajectory points
    %    - zoomed in on intersection

    % ██████╗  █████╗  ██████╗
    % ██╔══██╗██╔══██╗██╔════╝
    % ██║  ██║███████║██║  ███╗
    % ██║  ██║██╔══██║██║   ██║
    % ██████╔╝██║  ██║╚██████╔╝
    % ╚═════╝ ╚═╝  ╚═╝ ╚═════╝

    % TODO plot directed coupling graphs for different priority assignments, color nodes by level in coloring algo
    % constant_prioritizer = ConstantPrioritizer();
    % constant_prioritizer.current_priorities = [2 1 4 3 6 5 8 7];
    % coupling_directed = constant_prioritizer.prioritize(result.iteration_data(1));
    % max(kahn(coupling_directed))
    % figure
    % plot(digraph(coupling_directed))

end
