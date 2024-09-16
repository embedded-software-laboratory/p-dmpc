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

            % Cost
            cost_percent_average = data_cost_percent(experiment_results);

            export_plot( ...
                @series_plot_value, ...
                experiment_results, ...
                cost_percent_average, ...
                priority_names, ...
                export_fig_config = export_fig_config, ...
                file_path = sprintf('prioritization_cost_%s_%s.pdf', scenario, optimizer) ...
            );
            close all;

            % Plot computation time
            [~, time_med_approach_vehicle, ~, time_max_approach_vehicle] = data_time_approach_vehicle( ...
                experiment_results, ...
                computation_time_function = @data_time_prioritize_optimize_experiment ...
            );

            % Remove optimal priority, scale to ms
            series_time_max_ms = time_max_approach_vehicle(1:end - 1, :)' .* 1000;
            series_time_med_ms = time_med_approach_vehicle(1:end - 1, :)' .* 1000;

            fig = figure;
            series_plot_med_max( ...
                experiment_results, ...
                series_time_med_ms, ...
                series_time_max_ms, ...
                priority_names(1:end - 1), ...
                export_fig_config = ExportFigConfig.paper(paperheight = 6) ...
            )

            filename = sprintf('prioritization_time_%s_%s.pdf', scenario, optimizer);
            filepath = fullfile(FileNameConstructor.all_results(), filename);
            export_fig(fig, filepath);
            close all;

            % Plot computation levels
            [~, n_levels_med_approach_vehicle, ~, n_levels_max_approach_vehicle] = data_n_levels_approach_vehicle(experiment_results(:, 1:end - 1, :));

            fig = figure;
            series_plot_med_max( ...
                experiment_results, ...
                n_levels_med_approach_vehicle, ...
                n_levels_max_approach_vehicle, ...
                priority_names(1:end - 1), ...
                export_fig_config = ExportFigConfig.paper(paperheight = 6) ...
            );

            filename = sprintf('prioritization_levels_%s_%s.pdf', scenario, optimizer);
            filepath = fullfile(FileNameConstructor.all_results(), filename);
            export_fig(fig, filepath);
            close all;
        end

    end

end
