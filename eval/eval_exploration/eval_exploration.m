function eval_exploration(optional)

    arguments
        optional.cpm_lab_experiment_result (1, 1) string = fullfile(FileNameConstructor.all_results(), 'cpm_lab.mat')
    end

    %%
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

    % (n_vehicles x n_approaches x n_scenarios)
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
    series_time_max_ms = time_max_approach_vehicle' .* 1000;
    series_time_med_ms = time_med_approach_vehicle' .* 1000;

    fig = figure;
    series_plot_med_max( ...
        experiment_results, ...
        series_time_med_ms(:, 1:end - 1), ...
        series_time_max_ms(:, 1:end - 1), ...
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
    max_time = max(series_time_max_ms(:, 1:end - 1), [], "all");
    series_time_max_normalized = series_time_max_ms ./ max_time;
    series_time_med_normalized = series_time_med_ms ./ max_time;

    fig = figure;
    series_plot_med_max( ...
        experiment_results, ...
        series_time_med_normalized(:, 1:end - 1) * 100, ...
        series_time_max_normalized(:, 1:end - 1) * 100, ...
        priority_names(1:end - 1), ...
        ylabel = "$T_\mathrm{NCS}(p) / T_\mathrm{NCS}(p_\mathrm{constant})$ [\%]", ...
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

    %%
    % ████████╗██╗███╗   ███╗███████╗    ███████╗████████╗███████╗██████╗
    % ╚══██╔══╝██║████╗ ████║██╔════╝    ██╔════╝╚══██╔══╝██╔════╝██╔══██╗
    %    ██║   ██║██╔████╔██║█████╗      ███████╗   ██║   █████╗  ██████╔╝
    %    ██║   ██║██║╚██╔╝██║██╔══╝      ╚════██║   ██║   ██╔══╝  ██╔═══╝
    %    ██║   ██║██║ ╚═╝ ██║███████╗    ███████║   ██║   ███████╗██║
    %    ╚═╝   ╚═╝╚═╝     ╚═╝╚══════╝    ╚══════╝   ╚═╝   ╚══════╝╚═╝

    i_n_vehicles = 3; % 15 vehicles
    i_priority = 5; % Explorative priority
    i_scenario = 1;
    k = 1;
    export_plot( ...
        @plot_computation_time_for_step, ...
        experiment_results(i_n_vehicles, i_priority, i_scenario), ...
        k, ...
        export_fig_config = ExportFigConfig.paper(), ...
        file_path = fullfile(folderpath, 'computation_time_for_step.pdf') ...
    );
    close all;

    %%
    %  ██████╗██████╗ ███╗   ███╗    ██╗      █████╗ ██████╗
    % ██╔════╝██╔══██╗████╗ ████║    ██║     ██╔══██╗██╔══██╗
    % ██║     ██████╔╝██╔████╔██║    ██║     ███████║██████╔╝
    % ██║     ██╔═══╝ ██║╚██╔╝██║    ██║     ██╔══██║██╔══██╗
    % ╚██████╗██║     ██║ ╚═╝ ██║    ███████╗██║  ██║██████╔╝
    %  ╚═════╝╚═╝     ╚═╝     ╚═╝    ╚══════╝╚═╝  ╚═╝╚═════╝

    i_step_start = 21;
    cpm_lab_experiment_result = load(optional.cpm_lab_experiment_result).experiment_result;
    % ---
    % time per vehicle
    t_cpm_lab = data_time_prioritize_optimize_experiment(cpm_lab_experiment_result);
    % time NCS
    t_cpm_lab = max(t_cpm_lab, [], 1);
    % remove first time steps where code is loaded JIT
    t_cpm_lab = t_cpm_lab(i_step_start:end);

    t_cpm_lab_med = median(t_cpm_lab);
    t_cpm_lab_max = max(t_cpm_lab);

    %%
    % ███████╗██╗██╗     ███████╗
    % ██╔════╝██║██║     ██╔════╝
    % █████╗  ██║██║     █████╗
    % ██╔══╝  ██║██║     ██╔══╝
    % ██║     ██║███████╗███████╗
    % ╚═╝     ╚═╝╚══════╝╚══════╝
    filename = 'values.txt';
    filepath = fullfile(folderpath, filename);
    fileID = fopen(filepath, 'w');

    file_closer = onCleanup(@()fclose(fileID));

    for i_n_vehicles = 1:size(experiment_results, 1)
        fwrite(fileID, sprintf("%02i vehicles\n", experiment_results(i_n_vehicles, 1, 1).n_hlc));

        for i_approach = 1:size(experiment_results, 2)
            str_to_write = sprintf( ...
                "Prioritization+Optimization time for %20s -- max: %5.1f ms (%5.1f%%)-- med: %5.1f ms (%5.1f%%)\n" ...
                , experiment_results(i_n_vehicles, i_approach, 1).options.priority ...
                , time_max_approach_vehicle(i_approach, i_n_vehicles) * 1000 ...
                , series_time_max_normalized(i_n_vehicles, i_approach) * 100 ...
                , time_med_approach_vehicle(i_approach, i_n_vehicles) * 1000 ...
                , series_time_med_normalized(i_n_vehicles, i_approach) * 100 ...
            );
            fwrite(fileID, str_to_write);
        end

        fwrite(fileID, newline);

    end

    fwrite(fileID, newline);
    fwrite(fileID, newline);
    fwrite(fileID, newline);

    for i_n_vehicles = 1:size(experiment_results, 1)
        fwrite(fileID, sprintf("%02i vehicles\n", experiment_results(i_n_vehicles, 1, 1).n_hlc));

        for i_approach = 1:size(experiment_results, 2)
            str_to_write = sprintf( ...
                "Cost for %20s -- predicted %5.1f%% -- applied %5.1f%%\n" ...
                , experiment_results(i_n_vehicles, i_approach, 1).options.priority ...
                , cost_percent_average(i_n_vehicles, i_approach) ...
                , cost_percent_average_applied(i_n_vehicles, i_approach) ...
            );
            fwrite(fileID, str_to_write);
        end

        fwrite(fileID, newline);
    end

    % CPM Lab experiment
    fwrite(fileID, newline);
    fwrite(fileID, newline);
    fwrite(fileID, newline);

    str_to_write = sprintf( ...
        "CPM Lab -- max: %5.2f ms -- med: %5.2f ms\n" ...
        , t_cpm_lab_max * 1000 ...
        , t_cpm_lab_med * 1000 ...
    );
    fwrite(fileID, str_to_write);
end
