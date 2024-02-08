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
    %    PriorityStrategies.optimal_priority
                           ];

    priority_names = [
                      "$p_{\text{constant}}$"
                      "$p_{\text{random}}$"
                      "$p_{\text{color}}$"
                      "$p_{\text{constraint}}$"
                      "$p_{\text{explore}}$"
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
            lexendtext = priority_names;
            legend(lexendtext)
            % axes
            xlabel("$N_A$")
            ylabel( ...
                "$J_{NCS}(p) / J_{NCS}(p_\text{opt})$ [\%]", ...
                Interpreter = "latex" ...
            );

            ylim([-5, 105])

            for b = bar_handle
                xtips = b.XEndPoints;
                ytips = 50 * ones(size(b.YEndPoints));
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
            filename = sprintf('prioritization_cost_%s_%s.emf', scenario, optimizer);
            filepath = fullfile(FileNameConstructor.all_results(), filename);
            export_fig(fig, filepath);
            close all;

            % Plot computation time
        end

    end

end
