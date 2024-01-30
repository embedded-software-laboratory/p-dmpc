function eval_parallel_computation_CLs()
    %EVAL_PARALLEL_COMPUTATION_CLS Evaluate different number of computation levels
    disp('--------Prepare simulation data--------')
    options = Config();
    options.environment = Environment.Simulation;
    options.scenario_type = ScenarioType.commonroad;
    options.mpa_type = MpaType.single_speed;
    options.priority = 'STAC_priority';
    options.is_prioritized = true;
    options.constraint_from_successor = ConstraintFromSuccessor.area_of_standstill;
    options.constrained_enter_lanelet_crossing_area = false;
    options.dt_seconds = 0.2;
    options.options_plot_online.is_active = false;

    options.should_save_result = true;
    options.Hp = 7;
    options.T_end = 4;
    options.amount = 20;

    random_times = 10;
    CLs_s = [1, 2, 4, 6, options.amount];
    e_CLs = cell(random_times, length(CLs_s));
    experiment_results = cell(random_times, length(CLs_s));
    n_simulations = numel(e_CLs);
    count = 0;

    random_seed = RandStream('mt19937ar');

    for i = 1:random_times

        for j = 1:length(CLs_s)

            if i == 1 && j == 1
                options.should_reduce_result = false;
            else
                options.should_reduce_result = true;
            end

            options.max_num_CLs = CLs_s(j);
            options.path_ids = sort(randsample(random_seed, 1:40, options.amount), 'ascend');

            if FileNameConstructor.result_exists(options)
                disp('File already exists.')
            else
                % run simulation
                main(options);
            end

            % data processing
            e_CLs{i, j} = EvaluationParl(options);
            experiment_result = load_latest(options);
            experiment_results{i, j} = experiment_result;

            % display progress
            count = count + 1;
            disp(['--------Progress ' num2str(count) '/' num2str(n_simulations) ': done--------'])
        end

    end

    % get free flow speed, i.e., the speed that vehicles could travel if they are not influenced by others
    % vehicles
    free_flow_speed = get_free_flow_speed(options);
    disp('--------Free flow speed calculated--------')
    disp('--------Simulation data prepared--------')

    % get the path of the target folder to store figures
    [self_path, ~, ~] = fileparts(mfilename('fullpath')); % get the path of the current file
    idcs = strfind(self_path, filesep); % find all positions of '/'
    main_folder = self_path(1:idcs(end - 1) - 1); % two folders up, i.e., to main folder
    results_folder_path = fullfile(main_folder, 'results'); % results folder

    if ~isfolder(results_folder_path)
        % create target folder if not exist
        mkdir(results_folder_path)
    end

    disp('--------Ploting--------')
    plot_different_num_CLs(e_CLs, free_flow_speed, CLs_s, results_folder_path)
    disp('--------Plotted--------')

    % export video
    disp('--------Exporting frames--------')
    export_fig_for_video(experiment_results, results_folder_path)
    disp('--------Frames exported--------')

end

%% plot average speed and the actual number of computation levels
function plot_different_num_CLs(e_CLs, free_flow_speed, CLs_s, results_folder_path)
    close all

    %%% fig 1: Average speed (normalized)
    fig1 = figure();

    hold on
    box on
    grid on

    % maximum number of actual computation levels
    CLs_num_max = cellfun(@(c) c.CLs_num_max, e_CLs);

    X_string = string(CLs_s);
    %     X_string(1) = 'Parl.';
    %     X_string(2) = ['$N_{l,a}=$' num2str(CLs_s(2))];
    X_string(end) = '$\infty$';

    % plot average speed
    average_speed_s = cellfun(@(c) c.average_speed, e_CLs);
    average_speed_normalized = average_speed_s ./ free_flow_speed;

    boxplot(average_speed_normalized * 100, 'Colors', 'k', 'OutlierSize', 4, 'Labels', X_string);

    y_FFS = yline(free_flow_speed / free_flow_speed * 100, '--k', LineWidth = 0.6);

    legend(y_FFS, {'Free-flow speed'}, 'Location', 'south')
    ylabel('$\overline{v}\:[\%]$', Interpreter = 'latex')
    xlabel('Computation level limit')
    xaxis = get(gca, 'XAxis');
    xaxis.TickLabelInterpreter = 'latex'; % latex for x-axis

    ylim([58 102])
    yticks(60:10:100)

    % export fig 1
    export_config = ExportFigConfig.paper;
    export_config.paperwidth = 6;
    set_figure_properties(fig1, export_config)
    filepath = fullfile(results_folder_path, 'evalPaperCLsA.pdf');
    export_fig(fig1, filepath);
    disp(['A figure was saved under ' filepath '.'])
    close(fig1);

    %%% fig 2: Actual number of computation levels
    fig2 = figure();

    hold on
    box on
    grid on

    boxplot(CLs_num_max, 'Colors', 'k', 'OutlierSize', 4, 'Labels', X_string);

    level_max = 13;
    ylim([0 level_max])
    yticks(0:2:level_max)
    % xlabel('(b) Actual number of computation levels.')
    ylabel('$N_{l}$', Interpreter = 'latex')
    xaxis = get(gca, 'XAxis');
    xaxis.TickLabelInterpreter = 'latex'; % latex for x-axis

    % export fig 2
    export_config = ExportFigConfig.paper;
    export_config.paperwidth = 6;
    set_figure_properties(fig2, export_config)
    filepath = fullfile(results_folder_path, 'evalPaperCLsB.pdf');
    export_fig(fig2, filepath);
    disp(['A figure was saved under ' filepath '.'])
    close(fig2);
end

%% export videos
function export_fig_for_video(experiment_results, results_folder_path)
    i_CLs = 3; % number of computation levels: 4
    i_random = 1;
    experiment_result = experiment_results{i_random, i_CLs};

    experiment_result.options.options_plot_online.is_video_mode = true;
    experiment_result.options.options_plot_online.plot_coupling = true;
    experiment_result.options.options_plot_online.plot_weight = true;
    experiment_result.options.options_plot_online.plot_priority = true;

    % videoExportSetup.framerate = 30;
    timesteps = 14;

    for i_step = timesteps

        % after applying our approach
        experiment_result.options.options_plot_online.plot_weight = true;
        export_frame(experiment_result, i_step = i_step, frame_name = 'coupling_after_our_approach.png')
        % before applying our approach
        experiment_result.options.options_plot_online.plot_weight = false;
        export_frame(experiment_result, i_step = i_step, frame_name = 'coupling_before_our_approach.png')

        fig1 = figure();
        hold on
        plot_partitioned_graph(experiment_result, 'ShowWeights', true, 'ShowCutEdges', false, i_step = i_step);
        % title('Before applying our method')
        box on
        file_path_1 = fullfile(results_folder_path, 'graph_before_our_approach.png');
        exportgraphics(fig1, file_path_1, 'Resolution', 300)
        disp(['A figure was saved under ' file_path_1 '.'])

        fig2 = figure();
        hold on
        plot_partitioned_graph(experiment_result, 'ShowWeights', true, 'ShowCutEdges', true, i_step = i_step);
        % title('After applying our method')
        box on
        file_path_2 = fullfile(results_folder_path, 'graph_after_our_approach.png');
        exportgraphics(fig2, file_path_2, 'Resolution', 300)
        disp(['A figure was saved under ' file_path_2 '.'])

        close all
    end

end
