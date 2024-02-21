function eval_prioritization(optional)

    arguments
        optional.computation_mode (1, 1) ComputationMode = ComputationMode.sequential
    end

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

    % scenarios = [ScenarioType.commonroad, ScenarioType.circle];
    scenarios = ScenarioType.commonroad;
    % optimizers = [OptimizerType.MatlabOptimal, OptimizerType.MatlabSampled];
    optimizers = OptimizerType.MatlabOptimal;

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

            % Plot cost
            n_vehicles = [experiment_results(:, 1, 1).n_hlc];
            fig = figure;
            bar_handle = bar(n_vehicles, cost_percent_average);
            % legend
            legendtext = priority_names(1:end - 1);
            legend(legendtext, Location = 'best', Interpreter = 'latex');
            % axes
            xlabel("$N_A$")
            ylabel( ...
                "$J_\mathrm{NCS}(p) / J_\mathrm{NCS}(p_\mathrm{opt})$ [\%]", ...
                Interpreter = "latex" ...
            );

            for b = bar_handle
                xtips = b.XEndPoints;
                ytips = 60 * ones(size(b.YEndPoints));
                labels = arrayfun(@(s) sprintf("%5.1f", s), b.YData);
                text( ...
                    xtips, ...
                    ytips, ...
                    labels, ...
                    HorizontalAlignment = 'left', ...
                    VerticalAlignment = 'bottom', ...
                    Rotation = 55, ...
                    BackgroundColor = [1 1 1] ...
                );
            end

            set_figure_properties(fig, ExportFigConfig.presentation());
            filename = sprintf('prioritization_cost_%s_%s.pdf', scenario, optimizer);
            filepath = fullfile(FileNameConstructor.all_results(), filename);
            export_fig(fig, filepath);
            close all;

            % Plot computation time
            [~, time_med_approach_vehicle, ~, time_max_approach_vehicle] = data_time_approach_vehicle(experiment_results);

            n_vehicles = [experiment_results(:, 1, 1).n_hlc];
            fig = figure;
            bar(n_vehicles, time_max_approach_vehicle .* 1000);
            hold on
            bar(n_vehicles, time_med_approach_vehicle .* 1000);
            % legend

            str_max = "max ";
            str_med = "median ";
            legendtext = [ ...
                              strcat(repmat(str_max, length(priority_names), 1), priority_names) ...
                              strcat(repmat(str_med, length(priority_names), 1), priority_names) ...
                          ];
            legend(legendtext, Location = 'northeast', Interpreter = 'latex', NumColumns = 2);
            % axes
            xlabel('$N_{A}$', Interpreter = 'latex');
            ylabel('$T_{\mathrm{NCS}}$ [ms]', Interpreter = 'latex');

            set_figure_properties(fig, ExportFigConfig.presentation());
            rwth_colors_100 = rwth_color_order;
            rwth_colors_50 = rwth_color_order_50;
            colororder( ...
                fig, ...
                [rwth_colors_50(1:length(priority_names), :); ...
                 rwth_colors_100(1:length(priority_names), :)] ...
            );

            filename = sprintf('prioritization_time_%s_%s.pdf', scenario, optimizer);
            filepath = fullfile(FileNameConstructor.all_results(), filename);
            export_fig(fig, filepath);
            close all;

            % Plot computation levels
            [~, time_med_approach_vehicle, ~, time_max_approach_vehicle] = data_n_levels_approach_vehicle(experiment_results);

            n_vehicles = [experiment_results(:, 1, 1).n_hlc];
            fig = figure;
            bar(n_vehicles, time_max_approach_vehicle);
            hold on
            bar(n_vehicles, time_med_approach_vehicle);
            % legend

            str_max = "max ";
            str_med = "median ";
            legendtext = [ ...
                              strcat(repmat(str_max, length(priority_names), 1), priority_names) ...
                              strcat(repmat(str_med, length(priority_names), 1), priority_names) ...
                          ];
            legend(legendtext, Location = 'northeast', Interpreter = 'latex', NumColumns = 2);
            % axes
            xlabel('$N_{A}$', Interpreter = 'latex');
            ylabel('$N_{\mathrm{CL}}$', Interpreter = 'latex');

            set_figure_properties(fig, ExportFigConfig.presentation());
            rwth_colors_100 = rwth_color_order;
            rwth_colors_50 = rwth_color_order_50;
            colororder( ...
                fig, ...
                [rwth_colors_50(1:length(priority_names), :); ...
                 rwth_colors_100(1:length(priority_names), :)] ...
            );

            filename = sprintf('prioritization_levels_%s_%s.pdf', scenario, optimizer);
            filepath = fullfile(FileNameConstructor.all_results(), filename);
            export_fig(fig, filepath);
            close all;
        end

    end

end
