function eval_coloring_paper()
    %% intro
    fprintf("\nEvaluation script for \'Reducing Computation Time with Priority Assignment in Distributed Control\'\n\n")

    %% priority assignments <-> topological levels
    % Scenario at intersection with dedicated left turning lane:
    % sn(1)-ln(2)-re(3)-le(4)-ss(5)-ls(6)-rw(7)-lw(8)
    fprintf("\nEvaluate number of different priority assignments resulting in a certain number of levels in an 8-vehicle scenario at intersection\n")

    c = zeros(8, 8);
    c(2, 4) = 1; c(4, 2) = 1;
    c(4, 6) = 1; c(6, 4) = 1;
    c(6, 8) = 1; c(8, 6) = 1;
    c(8, 2) = 1; c(2, 8) = 1;
    c(5, 2) = 1; c(2, 5) = 1;
    c(5, 3) = 1; c(3, 5) = 1;
    c(5, 4) = 1; c(4, 5) = 1;
    c(1, 8) = 1; c(8, 1) = 1;
    c(1, 7) = 1; c(7, 1) = 1;
    c(1, 6) = 1; c(6, 1) = 1;

    lvl_dist = calc_level_dist(c);

    % plot & save figure
    fig = figure();
    histogram(lvl_dist, FaceAlpha = 1);
    set(gca, 'yscale', 'log');
    set(gca, 'XTick', 1:size(c, 1));
    ylabel('\# Prio. Assignments', Interpreter = 'LaTex');
    xlabel('Computation Levels', Interpreter = 'LaTex');
    set_figure_properties(fig, ExportFigConfig.paper());
    mkdir('./results')
    export_fig(fig, fullfile('./results/levels.pdf'));
    close(fig);

    %% computation time <-> topological levels
    % Scenario at intersection with dedicated left turning lane:
    % sn(1)-ln(2)-re(3)-le(4)-ss(5)-ls(6)-rw(7)-lw(8)
    fprintf("\nEvaluate computation time for planning using priority assignments with different numbers of levels\n")
    level = 8:-5:3;
    tcomp_t = 100 * level / 8;

    % plot & save figure
    fig = figure(Position = [100 100 600 630], Color = [1 1 1]);
    barh(level, tcomp_t);
    yticklabels({'$p_c$', '$p_b$'})
    set(gca, 'XTick', 0:20:100);
    xline(min(tcomp_t), '-r');
    xlabel('Computation Time [\%]', Interpreter = 'LaTex');
    ylabel('Priority Assignment', Interpreter = 'LaTex');
    set(gca, 'TickLabelInterpreter', 'latex');
    set_figure_properties(fig, ExportFigConfig.paper());
    export_fig(fig, fullfile('./results/tcomp.pdf'));
    close(fig);

    %% computation time algorithm
    % theoretical
    fprintf("\nEvaluate computation time for priority assignment algorithm in random scenarios of varying size\n")
    n_runs = 10;
    n_agents = [5, 10:10:50, 100:50:500, 600:100:1000];
    range = 1:length(n_agents);
    tcomp = zeros(1, length(n_agents));
    tcomp_helper = zeros(1, n_runs);

    prioritizer = ColoringPrioritizer();
    iter = struct;

    for n = range
        adjacency = triu(randi([0 1], n_agents(n), n_agents(n)), 1);
        adjacency = adjacency + adjacency';

        iter.adjacency = adjacency;

        for i = 1:n_runs
            tstart = tic;
            prioritizer.prioritize(iter, [], [], []);
            tcomp_helper(i) = toc(tstart);
        end

        tcomp(n) = median(tcomp_helper);
    end

    % plot & save figure
    fig = figure(Position = [100 100 600 630], Color = [1 1 1]);
    plot(n_agents, tcomp, '-o', Color = '#0072BD');
    scatter(n_agents, tcomp, 10, 'filled')
    set(gca, 'yscale', 'log');
    xlabel('\# Agents', Interpreter = 'LaTex')
    ylabel('Computation Time [s]', Interpreter = 'LaTex')
    set(gca, 'YTick', 10.^(-5:2:1));
    set_figure_properties(fig, ExportFigConfig.paper());
    export_fig(fig, fullfile('./results/coloring_time.pdf'));
    close(fig);

    % Scenario at intersection with dedicated left turning lane:
    % sn(1)-ln(2)-re(3)-le(4)-ss(5)-ls(6)-rw(7)-lw(8)
    fprintf("\nEvaluate computation time for priority assignment algorithm in 8-vehicle scenario at intersection\n")
    n = 100;
    tcomp_s = zeros(1, n);
    iter.adjacency = c;

    for i = 1:n
        tstart = tic;
        prioritizer.prioritize(iter, [], [], []);
        tcomp_s(i) = toc(tstart);
    end

    fprintf("\nComputation Time - Coloring Algorithm - in 8-Vehicle-Intersection-Scenario\n\n")
    fprintf("Max: %.5f\n", max(tcomp_s))
    fprintf("Mean: %.5f\n", mean(tcomp_s))
    fprintf("Median: %.5f\n\n", median(tcomp_s))

    %% Comparison to other prioritizing algorithms
    fprintf( ...
        "\nCompare computation levels from different prioritizing algorithms\n" ...
    )
    priority_assignment_algorithms = {
                                      'FCA_priority'
                                      'random_priority'
                                      'constant_priority'
                                      'coloring_priority'
                                      };

    options = Config;
    options.mpa_type = MpaType.single_speed;
    options.T_end = 180;
    options.Hp = 8;
    options.is_prioritized = true;
    options.environment = Environment.Simulation;
    options.options_plot_online.is_active = false;
    options.constraint_from_successor = ConstraintFromSuccessor.area_of_standstill;
    options.constrained_enter_lanelet_crossing_area = false; % false: no constraint on entering the crossing area
    options.should_save_result = 1;
    options.should_reduce_result = 1;
    options.scenario_type = ScenarioType.commonroad;

    nsVeh = 1:20;
    % number of different random scenarios per priority assignment and #vehicles
    seeds = 1:9;

    scenarios = commonroad_random(options, nsVeh, seeds);

    results = run_scenario_with_priority_algorithm( ...
        scenarios, priority_assignment_algorithms ...
    );

    % plot Computation levels histogram excluding deadlock
    plot_levels(results);

    % plot lanelets map
    figure_handle = figure();

    plot_lanelets(scenarios(1).road_raw_data.lanelet);
    axis equal;
    xlabel('$x$ [m]', Interpreter = 'latex');
    ylabel('$y$ [m]', Interpreter = 'latex');
    set_figure_properties(figure_handle, ExportFigConfig.paper('paperheight', 6));
    export_fig(figure_handle, './results/lab_map.pdf');
    close(figure_handle);
end
