function eval_exploration()
    foldername = 'exploration';
    folderpath = fullfile(FileNameConstructor.all_results(), foldername);

    %%
    % ███████╗██╗  ██╗██████╗ ███████╗██████╗ ██╗███╗   ███╗███████╗███╗   ██╗████████╗
    % ██╔════╝╚██╗██╔╝██╔══██╗██╔════╝██╔══██╗██║████╗ ████║██╔════╝████╗  ██║╚══██╔══╝
    % █████╗   ╚███╔╝ ██████╔╝█████╗  ██████╔╝██║██╔████╔██║█████╗  ██╔██╗ ██║   ██║
    % ██╔══╝   ██╔██╗ ██╔═══╝ ██╔══╝  ██╔══██╗██║██║╚██╔╝██║██╔══╝  ██║╚██╗██║   ██║
    % ███████╗██╔╝ ██╗██║     ███████╗██║  ██║██║██║ ╚═╝ ██║███████╗██║ ╚████║   ██║
    % ╚══════╝╚═╝  ╚═╝╚═╝     ╚══════╝╚═╝  ╚═╝╚═╝╚═╝     ╚═╝╚══════╝╚═╝  ╚═══╝   ╚═╝

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

    experiment_results = eval_experiments( ...
        computation_mode = ComputationMode.parallel_physically ...
        , optimizer = OptimizerType.MatlabSampled ...
        , priority_strategies = priority_strategies ...
    );

    % Copy to evaluation-specific folder
    for experiment_result = experiment_results(:)'
        file_path = char(FileNameConstructor.experiment_result_folder_path( ...
            experiment_result.options ...
        ));
        result_str_pos = strfind(file_path, 'results') + numel('results');
        file_path = [ ...
                         folderpath, filesep, ...
                         file_path(result_str_pos + 1:end), filesep, ...
                         '999999-999999.mat' ...
                     ];
        [is_folder_created, ~] = mkdir(fileparts(file_path));
        assert(is_folder_created)
        save(file_path, "experiment_result")
    end

    %%
    % ███╗   ███╗██████╗  █████╗
    % ████╗ ████║██╔══██╗██╔══██╗
    % ██╔████╔██║██████╔╝███████║
    % ██║╚██╔╝██║██╔═══╝ ██╔══██║
    % ██║ ╚═╝ ██║██║     ██║  ██║
    % ╚═╝     ╚═╝╚═╝     ╚═╝  ╚═╝
    figure;
    export_plot(@plot_mpa, experiment_results(1), file_path = fullfile(folderpath, 'mpa.pdf'));
    close all;

    %%
    %  ██████╗ ██████╗ ███████╗████████╗
    % ██╔════╝██╔═══██╗██╔════╝╚══██╔══╝
    % ██║     ██║   ██║███████╗   ██║
    % ██║     ██║   ██║╚════██║   ██║
    % ╚██████╗╚██████╔╝███████║   ██║
    %  ╚═════╝ ╚═════╝ ╚══════╝   ╚═╝

    cost_percent_average = data_cost_percent(experiment_results);

    export_plot( ...
        @series_plot_value, ...
        experiment_results, ...
        cost_percent_average, ...
        priority_names, ...
        file_path = fullfile(folderpath, 'prioritization_cost.pdf') ...
    );
    close all;

    % Applied cost
    cost_percent_average_applied = data_cost_percent( ...
        experiment_results, ...
        function_name = @data_cost_applied_experiment ...
    );

    export_plot( ...
        @series_plot_value, ...
        experiment_results, ...
        cost_percent_average_applied, ...
        priority_names, ...
        file_path = fullfile(folderpath, 'prioritization_cost_applied.pdf') ...
    );
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
    );
    export_fig(fig, fullfile(folderpath, 'prioritization_time.pdf'));
    close all;

    %%
    % ████████╗██╗███╗   ███╗███████╗    ███╗   ██╗ ██████╗ ██████╗ ███╗   ███╗
    % ╚══██╔══╝██║████╗ ████║██╔════╝    ████╗  ██║██╔═══██╗██╔══██╗████╗ ████║
    %    ██║   ██║██╔████╔██║█████╗      ██╔██╗ ██║██║   ██║██████╔╝██╔████╔██║
    %    ██║   ██║██║╚██╔╝██║██╔══╝      ██║╚██╗██║██║   ██║██╔══██╗██║╚██╔╝██║
    %    ██║   ██║██║ ╚═╝ ██║███████╗    ██║ ╚████║╚██████╔╝██║  ██║██║ ╚═╝ ██║██╗
    %    ╚═╝   ╚═╝╚═╝     ╚═╝╚══════╝    ╚═╝  ╚═══╝ ╚═════╝ ╚═╝  ╚═╝╚═╝     ╚═╝╚═╝
    max_time = max(series_time_max_ms, [], "all");
    series_time_max_normalized = series_time_max_ms ./ max_time;
    series_time_med_normalized = series_time_med_ms ./ max_time;

    fig = figure;
    series_plot_med_max( ...
        experiment_results, ...
        series_time_med_normalized, ...
        series_time_max_normalized, ...
        priority_names(1:end - 1), ...
        export_fig_config = ExportFigConfig.paper(paperheight = 6) ...
    );
    export_fig(fig, fullfile(folderpath, 'prioritization_time_normalized.pdf'));
    close all;

    %%
    % ██╗     ███████╗██╗   ██╗███████╗██╗     ███████╗
    % ██║     ██╔════╝██║   ██║██╔════╝██║     ██╔════╝
    % ██║     █████╗  ██║   ██║█████╗  ██║     ███████╗
    % ██║     ██╔══╝  ╚██╗ ██╔╝██╔══╝  ██║     ╚════██║
    % ███████╗███████╗ ╚████╔╝ ███████╗███████╗███████║
    % ╚══════╝╚══════╝  ╚═══╝  ╚══════╝╚══════╝╚══════╝

    [~, n_levels_med_approach_vehicle, ~, n_levels_max_approach_vehicle] = data_n_levels_approach_vehicle(experiment_results(:, 1:end - 1, :));

    fig = figure;
    series_plot_med_max( ...
        experiment_results, ...
        n_levels_med_approach_vehicle, ...
        n_levels_max_approach_vehicle, ...
        priority_names(1:end - 1), ...
        export_fig_config = ExportFigConfig.paper(paperheight = 6) ...
    );
    export_fig(fig, fullfile(folderpath, 'n_levels.pdf'));
    close all;

    %     %%
    %     % ███████╗██╗██╗     ███████╗
    %     % ██╔════╝██║██║     ██╔════╝
    %     % █████╗  ██║██║     █████╗
    %     % ██╔══╝  ██║██║     ██╔══╝
    %     % ██║     ██║███████╗███████╗
    %     % ╚═╝     ╚═╝╚══════╝╚══════╝
    %     filename = 'prioritization_coloring.txt';
    %     filepath = fullfile(folderpath, filename);
    %     fileID = fopen(filepath, 'w');

    %     for i = 1:numel(experiment_results)
    %         str_to_write = sprintf( ...
    %             "Prioritization+Optimization time for %17s -- max: %5.2f ms -- med: %5.2f ms\n" ...
    %             , experiment_results(i).options.priority ...
    %             , time_max_approach_vehicle(i) * 1000 ...
    %             , time_med_approach_vehicle(i) * 1000 ...
    %         );
    %         fwrite(fileID, str_to_write);
    %     end

    %     fwrite(fileID, newline);

    %     for experiment_result = experiment_results
    %         prioritize_timing = vertcat(experiment_result.timing.prioritize);
    %         prioritize_duration = max(prioritize_timing(2:2:end, :), [], 1);
    %         prioritize_duration_max = max(prioritize_duration) * 1000;
    %         prioritize_duration_med = median(prioritize_duration) * 1000;
    %         str_to_write = sprintf( ...
    %             "Prioritization time for %17s -- max: %5.2f ms -- med: %5.2f ms\n" ...
    %             , experiment_result.options.priority ...
    %             , prioritize_duration_max ...
    %             , prioritize_duration_med ...
    %         );
    %         fwrite(fileID, str_to_write);
    %     end

    %     fwrite(fileID, newline);

    %     for i = 1:numel(experiment_results)
    %         str_to_write = sprintf( ...
    %             "Prioritization cost for %17s -- %5.1f %%\n" ...
    %             , experiment_results(i).options.priority ...
    %             , cost_percent_average(i) ...
    %         );
    %         fwrite(fileID, str_to_write);
    %     end

    % % TODO cost from cost_percent_average_applied
    %     fclose(fileID);
end
