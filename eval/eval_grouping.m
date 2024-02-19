function eval_grouping(optional)

    arguments
        optional.computation_mode (1, 1) ComputationMode = ComputationMode.sequential
    end

    max_num_CLs = [1, 2, 4, 6, 99];

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
                max_num_CLs = max_num_CLs ...
            );
            save('experiment_results_grouping.mat', 'experiment_results')

            % Plot cost
            % Process results
            cost_percent_average = data_cost_percent(experiment_results);

            % plot
            n_vehicles = [experiment_results(:, 1, 1).n_hlc];
            fig = figure;
            bar_handle = bar(n_vehicles, cost_percent_average);
            % legend
            legendtext = string(max_num_CLs(1:end - 1));
            legendtext = arrayfun( ...
                @(element) strcat("$N_\mathrm{CL} = ", element, "$"), ...
                legendtext ...
            );
            legend(legendtext)
            % axes
            xlabel("$N_A$")
            ylabel( ...
                "$J_\mathrm{NCS}(N_\mathrm{CL}) / J_\mathrm{NCS}(N_{\mathrm{CL},\infty})$ [\%]", ...
                Interpreter = "latex" ...
            );

            % ylim([-5, 105])
            yscale("log");
            ymax = ylim;
            ymax = ymax(2);
            ymin = 1e1;
            ylim([ymin, ymax])

            for i_num_vehicles = 1:length(n_vehicles)
                num_cls_infty = max( ...
                    [experiment_results(i_num_vehicles, end, :).max_number_of_computation_levels] ...
                );
                label = strcat("$N_{\mathrm{CL},\infty} = ", sprintf("%d", num_cls_infty), "$");

                y_text = ymin * 1.2;

                text( ...
                    n_vehicles(i_num_vehicles), ...
                    y_text, ...
                    label, ...
                    BackgroundColor = [1 1 1], ...
                    HorizontalAlignment = "center", ...
                    VerticalAlignment = "bottom", ...
                    Interpreter = "latex" ...
                );

            end

            for b = bar_handle
                xtips = b.XEndPoints;
                ytips = 40 * ones(size(b.YEndPoints));
                labels = arrayfun(@(s) sprintf("%5.1f", s), b.YData);
                text( ...
                    xtips, ...
                    ytips, ...
                    labels, ...
                    HorizontalAlignment = 'left', ...
                    VerticalAlignment = 'middle', ...
                    Rotation = 55, ...
                    BackgroundColor = [1 1 1] ...
                );
            end

            set_figure_properties(fig, ExportFigConfig.presentation());
            filename = sprintf('grouping_cost_%s_%s.pdf', scenario, optimizer);
            filepath = fullfile(FileNameConstructor.all_results(), filename);
            export_fig(fig, filepath);
            filename = sprintf('grouping_cost_%s_%s.emf', scenario, optimizer);
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
            % legend
            legendtext = string(max_num_CLs);
            legendtext = arrayfun( ...
                @(element) strcat("$N_\mathrm{CL} = ", element, "$"), ...
                legendtext ...
            )';
            legendtext = [ ...
                              strcat(repmat(str_max, length(legendtext), 1), legendtext) ...
                              strcat(repmat(str_med, length(legendtext), 1), legendtext) ...
                          ];
            legend(legendtext, Location = 'northeast', Interpreter = 'latex', NumColumns = 2);
            % axes
            xlabel('$N_{\mathrm{A}}$', Interpreter = 'latex');
            ylabel('$T_{\mathrm{NCS}}$ [ms]', Interpreter = 'latex');

            for i_num_vehicles = 1:length(n_vehicles)
                num_cls_infty = max( ...
                    [experiment_results(i_num_vehicles, end, :).max_number_of_computation_levels] ...
                );
                label = strcat("$N_{\mathrm{CL},\infty} = ", sprintf("%d", num_cls_infty), "$");

                y_text = ymin * 1.2;

                text( ...
                    n_vehicles(i_num_vehicles), ...
                    y_text, ...
                    label, ...
                    BackgroundColor = [1 1 1], ...
                    HorizontalAlignment = "center", ...
                    VerticalAlignment = "bottom", ...
                    Interpreter = "latex" ...
                );

            end

            set_figure_properties(fig, ExportFigConfig.presentation());
            rwth_colors_100 = rwth_color_order;
            rwth_colors_50 = rwth_color_order_50;
            colororder( ...
                fig, ...
                [rwth_colors_50(1:length(max_num_CLs), :); ...
                 rwth_colors_100(1:length(max_num_CLs), :)] ...
            );

            filename = sprintf('grouping_time_%s_%s.pdf', scenario, optimizer);
            filepath = fullfile(FileNameConstructor.all_results(), filename);
            export_fig(fig, filepath);
            filename = sprintf('grouping_time_%s_%s.emf', scenario, optimizer);
            filepath = fullfile(FileNameConstructor.all_results(), filename);
            export_fig(fig, filepath);
            close all;
        end

    end

end
