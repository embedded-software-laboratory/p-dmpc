function eval_prioritization(optional)

    arguments
        optional.computation_mode (1, 1) ComputationMode = ComputationMode.parallel
        optional.scenarios (1, :) ScenarioType = [ScenarioType.commonroad, ScenarioType.circle]
        optional.optimizers (1, :) OptimizerType = [OptimizerType.MatlabOptimal, OptimizerType.MatlabSampled]
    end

    export_fig_config = ExportFigConfig.paper(paperheight = 6);
    priority_strategies = [
                           PriorityStrategies.constant_priority
                           PriorityStrategies.random_priority
                           PriorityStrategies.coloring_priority
                           PriorityStrategies.FCA_priority
                           PriorityStrategies.explorative_priority
                           PriorityStrategies.optimal_priority
                           ];

    priority_names = [
                      "$p_{\mathrm{constant}}$"
                      "$p_{\mathrm{random}}$"
                      "$p_{\mathrm{color}}$"
                      "$p_{\mathrm{constraint}}$"
                      "$p_{\mathrm{explore}}$"
                      "$p_{\mathrm{optimal}}$"
                      ];

    scenarios = optional.scenarios;
    optimizers = optional.optimizers;

    for scenario = scenarios

        for optimizer = optimizers
            % Generate results
            experiment_results = eval_experiments( ...
                computation_mode = optional.computation_mode, ...
                scenario_type = scenario, ...
                optimizer = optimizer, ...
                priority_strategies = priority_strategies ...
            );

            save('experiment_results_prioritization.mat', 'experiment_results')
            % Process results
            cost_percent_average = data_cost_percent(experiment_results);

            export_plot( ...
                @series_plot_cost_percent, ...
                experiment_results, ...
                cost_percent_average, ...
                priority_names, ...
                export_fig_cfg = export_fig_config, ...
                file_path = sprintf('prioritization_cost_%s_%s.pdf', scenario, optimizer) ...
            );
            close all;

            % Plot computation time
            [~, time_med_approach_vehicle, ~, time_max_approach_vehicle] = data_time_approach_vehicle( ...
                experiment_results(:, 1:end - 1, :), ...
                computation_time_function = @data_time_prioritize_optimize_experiment ...
            );

            n_vehicles = [experiment_results(:, 1, 1).n_hlc];
            fig = figure;
            max_bar = bar(n_vehicles, time_max_approach_vehicle' .* 1000);
            hold on
            med_bar = bar(n_vehicles, time_med_approach_vehicle' .* 1000);

            % legend
            str_med = "med ";
            str_max = "max ";
            legendtext = [ ...
                              strcat(repmat(str_med, length(priority_names) - 1, 1), priority_names(1:end - 1)) ...
                              strcat(repmat(str_max, length(priority_names) - 1, 1), priority_names(1:end - 1)) ...
                          ];
            legend([med_bar, max_bar], legendtext, Location = 'northoutside', Interpreter = 'latex', NumColumns = 2);
            % axes
            xlabel('$N_{A}$', Interpreter = 'latex');
            ylabel('$T_{\mathrm{NCS}}$ [ms]', Interpreter = 'latex');

            set_figure_properties(fig, export_fig_config);
            legend(Box = 'off')
            rwth_colors_100 = rwth_color_order;
            rwth_colors_50 = rwth_color_order_50;
            colororder( ...
                fig, ...
                [rwth_colors_50(1:length(priority_names) - 1, :); ...
                 rwth_colors_100(1:length(priority_names) - 1, :)] ...
            );

            filename = sprintf('prioritization_time_%s_%s.pdf', scenario, optimizer);
            filepath = fullfile(FileNameConstructor.all_results(), filename);
            export_fig(fig, filepath);
            close all;

            % Plot computation levels
            [~, time_med_approach_vehicle, ~, time_max_approach_vehicle] = data_n_levels_approach_vehicle(experiment_results(:, 1:end - 1, :));

            n_vehicles = [experiment_results(:, 1, 1).n_hlc];
            fig = figure;
            max_bar = bar(n_vehicles, time_max_approach_vehicle');
            hold on
            med_bar = bar(n_vehicles, time_med_approach_vehicle');

            % legend
            legend([med_bar, max_bar], legendtext, Location = 'northoutside', Interpreter = 'latex', NumColumns = 2);
            % axes
            xlabel('$N_{A}$', Interpreter = 'latex');
            ylabel('$N_{\mathrm{CL}}$', Interpreter = 'latex');

            set_figure_properties(fig, export_fig_config);
            legend(Box = 'off')
            rwth_colors_100 = rwth_color_order;
            rwth_colors_50 = rwth_color_order_50;
            colororder( ...
                fig, ...
                [rwth_colors_50(1:length(priority_names) - 1, :); ...
                 rwth_colors_100(1:length(priority_names) - 1, :)] ...
            );

            filename = sprintf('prioritization_levels_%s_%s.pdf', scenario, optimizer);
            filepath = fullfile(FileNameConstructor.all_results(), filename);
            export_fig(fig, filepath);
            close all;
        end

    end

end
