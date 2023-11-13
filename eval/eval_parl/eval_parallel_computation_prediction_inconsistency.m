function results = eval_parallel_computation_prediction_inconsistency()
    %EVAL_PARALLEL_COMPUTATION_PREDICTION_INCONSISTENCY Evaluate of the strategy dealing with the prediction inconsistency problem in parallel
    %computation paper
    disp('--------Prepare simulation data--------')
    isDealPredictionInconsistency = [true, false];
    fallback_type = {'localFallback', 'noFallback'};
    % prepare simulation options
    options = Config();
    options.environment = Environment.Simulation;
    options.scenario_type = ScenarioType.commonroad;
    options.mpa_type = MpaType.single_speed;
    options.Hp = 5;

    options.T_end = 4;
    options.dt_seconds = 0.2;
    options.max_num_CLs = 1;
    options.priority = 'STAC_priority';
    options.is_prioritized = true;
    options.compute_in_parallel = false;
    options.should_save_result = true;
    options.should_reduce_result = false;
    options.strategy_consider_veh_without_ROW = '1';
    options.strategy_enter_lanelet_crossing_area = '1';
    options.is_free_flow = false;

    % TODO create fitting scenario
    % options.reference_path_struct.lanelets_index = {[95, 69, 64, 62, 75, 55, 53], [76, 24, 13, 15, 3, 5], [12, 73, 92, 94, 100, 101]};
    % options.reference_path_struct.start_point = [5, 1, 11];
    % options.amount = length(options.reference_path_struct.lanelets_index);
    options.path_ids = 1:options.amount;
    results = cell(length(isDealPredictionInconsistency), options.amount);

    for i = 1:length(isDealPredictionInconsistency)
        options.isDealPredictionInconsistency = isDealPredictionInconsistency(i);
        options.fallback_type = fallback_type{i};

        for veh_id = 1:options.amount
            full_path = FileNameConstructor.get_results_full_path(options, veh_id);

            if isfile(full_path)
                disp('File already exists.')
            else
                % run simulation
                main(options);
            end

            load(full_path, 'experiment_result');
            results{i, veh_id} = experiment_result;
        end

    end

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

    rwth_colors = rwth_color_order();
    veh_colors = rwth_colors(1:options.amount, :);
    x_lim = [1.6 3.4];
    y_lim = [1.0 2.5];
    disp('--------Plot--------')
    plot_prediction_inconsistency_addressed(results, results_folder_path)
    plot_prediction_inconsistency_not_addressed(results, results_folder_path)
    disp('--------Plotted--------')

    % export video
    answer = questdlg('Would you like to export videos (may take several minutes)?', ...
        'Export videos?', ...
        'Yes', 'No', 'No');

    if strcmp(answer, 'Yes')
        disp('--------Preparing simulation data for video--------')
        % combine distuibuted simulation data
        results_combined = combine_distributed_results(results);
        disp('--------Exporting videos--------')
        export_videos(results_combined, veh_colors, x_lim, y_lim, results_folder_path)
        disp('--------Videos exported--------')
    end

end

%% subfunction
function plot_prediction_inconsistency_addressed(results, results_folder_path)
    close all
    plot_footprints(1, results, results_folder_path);
    plot_viewpoint_reachable_set(1, results, results_folder_path);
    plot_actual_plans(1, results, results_folder_path);
end

%% subfunction 2
function plot_prediction_inconsistency_not_addressed(results, results_folder_path)
    close all
    plot_footprints(2, results, results_folder_path);
    plot_viewpoint_previous_occupancy(2, results, results_folder_path);
    plot_actual_plans(2, results, results_folder_path);
end

%% helper function
function p = plot_cell_arrays(cells, color, isFill)
    % Plot shapes contained in a cell array
    %     CM = jet(Hp); % colormap
    hold on

    if nargin == 1
        isFill = false;
    end

    if isFill

        for j = size(cells, 2):-1:1
            shape = cells{j};
            patch(shape(1, :), shape(2, :), color, 'FaceAlpha', (1 - j / (size(cells, 2) + 2)) * 0.5);
        end

    else
        CM = rwth_color_order(size(cells, 2) + 3);
        CM = CM(4:end, :);

        for j = 1:size(cells, 2)
            shape = cells{j};
            p(j) = plot(shape(1, :), shape(2, :), 'LineWidth', 1, 'Color', CM(j, :), 'LineStyle', '-.');
        end

    end

end

function results_combined = combine_distributed_results(results)
    results_combined = cell(size(results, 1), 1);

    for i = 1:size(results, 1)
        experiment_result = results{i, 1};

        for j = 2:size(results, 2)
            trajectory_predictions = {results{i, j}.trajectory_predictions{j, :}};
            [experiment_result.trajectory_predictions{j, :}] = trajectory_predictions{:};

            for k = 1:length(results{i, j}.iteration_data)
                experiment_result.iteration_data{k}.referenceTrajectoryPoints(j, :, :) = results{i, j}.iteration_data{k}.referenceTrajectoryPoints(j, :, :);
                experiment_result.iteration_data{k}.referenceTrajectoryIndex(j, :) = results{i, j}.iteration_data{k}.referenceTrajectoryIndex(j, :);
                experiment_result.iteration_data{k}.v_ref(j, :) = results{i, j}.iteration_data{k}.v_ref(j, :);
            end

        end

        results_combined{i} = experiment_result;
    end

end

% export videos
function export_videos(results, veh_colors, x_lim, y_lim, results_folder_path)

    for i = 1:numel(results)
        experiment_result = results{i};

        experiment_result.options.options_plot_online.is_custom_colors = true; % FIXME
        experiment_result.options.options_plot_online.custom_colors = veh_colors; % FIXME
        experiment_result.options.plot_limits(1, :) = x_lim;
        experiment_result.options.plot_limits(2, :) = y_lim;
        experiment_result.options.options_plot_online.plot_xy_labels = false; % FIXME
        experiment_result.options.options_plot_online.plot_xy_ticks = false; % FIXME
        experiment_result.options.options_plot_online.is_dynamic_colors = false; % FIXME

        experiment_result.options.options_plot_online.plot_scenario_name = false; % FIXME
        experiment_result.options.options_plot_online.plot_timesteps = true; % FIXME

        experiment_result.options.options_plot_online.is_video_mode = true;
        experiment_result.options.options_plot_online.plot_coupling = true;
        experiment_result.options.options_plot_online.plot_weight = false;
        experiment_result.options.options_plot_online.plot_priority = false;
        approaches = {'reachable_set', 'previous_plans'};
        % TODO custom plot functions
        if i == 1
            % plot the reachable sets of vehicles 1 and 2
            experiment_result.options.options_plot_online.plot_reachable_sets = true;
            experiment_result.options.options_plot_online.vehicles_reachable_sets = [1, 2];
            experiment_result.options.options_plot_online.plot_predicted_occupancy = true; % FIXME
            experiment_result.options.options_plot_online.vehicles_predicted_occupancy = [3]; % FIXME
            export_video(experiment_result, framerate = 30, name = fullfile(results_folder_path, [approaches{i}, '-view_vehicle_3.mp4']))
            % plot the actual plans of vehicles 1 and 2
            experiment_result.options.options_plot_online.plot_reachable_sets = false;
            experiment_result.options.options_plot_online.plot_predicted_occupancy = true; % FIXME
            experiment_result.options.options_plot_online.vehicles_predicted_occupancy = [1, 2, 3]; % FIXME
            export_video(experiment_result, framerate = 30, name = fullfile(results_folder_path, [approaches{i}, '-actual_plans.mp4']))
        else
            % plot the previously predicted occupancies of vehicles 1 and 2
            experiment_result.options.options_plot_online.plot_predicted_occupancy_previous = true; % FIXME
            experiment_result.options.options_plot_online.vehicles_predicted_occupancy_previous = [1, 2]; % FIXME
            experiment_result.options.options_plot_online.plot_predicted_occupancy = true; % FIXME
            experiment_result.options.options_plot_online.vehicles_predicted_occupancy = [3]; % FIXME
            export_video(experiment_result, framerate = 30, name = fullfile(results_folder_path, [approaches{i}, '-view_vehicle_3.mp4']))
            % plot the actual plans of vehicles 1 and 2
            experiment_result.options.options_plot_online.plot_predicted_occupancy_previous = false; % FIXME
            experiment_result.options.options_plot_online.plot_predicted_occupancy = true; % FIXME
            experiment_result.options.options_plot_online.vehicles_predicted_occupancy = [1, 2, 3]; % FIXME
            export_video(experiment_result, framerate = 30, name = fullfile(results_folder_path, [approaches{i}, '-actual_plans.mp4']))
        end

    end

end

function plot_predicted_occupancy(trajectory_predictions, options, scenario, color, trims_stop, is_one_step_shifted)

    % PLOT_PREDICTED_OCCUPANCY Plot predicted occupied areas
    % Input
    %   trajectory_predictions: predicted trajectory
    %   options: instance of the class "Config"
    %   scenario: instance of the class "Scenario"
    %   color: define color of the area
    %   trims_step: equlibrium trim(s)
    %   is_one_step_shifted: true/false, if true, one-step-shifted occupancy
    %   will be shown
    %
    n_predictions = size(trajectory_predictions, 1);
    n_ticks = n_predictions / options.Hp;

    i = 0;

    for tick = n_predictions - n_ticks + 1:-n_ticks:1

        if is_one_step_shifted && tick == 1
            % ignore the first timestep
            continue
        end

        i = i + 1;
        x = trajectory_predictions(tick, :);
        trim1 = trajectory_predictions(tick, 4);

        if i == 1
            assert(length(trims_stop) == 1)
            trim2 = trims_stop; % because of recursive feasibility, the last trim must be the equilibrium trim (speed=0)
        else
            trim2 = trajectory_predictions(tick + n_ticks, 4);
        end

        area = scenario.mpa.maneuvers{trim1, trim2}.area; % FIXME scenario.mpa
        [area_x, area_y] = translate_global(x(3), x(1), x(2), area(1, :), area(2, :));
        area_poly = polyshape([area_x; area_y]');
        plot(area_poly, 'FaceColor', color, 'FaceAlpha', (i / (options.Hp + 2)) * 0.5)
    end

end

function plot_footprints(i_result, results, results_folder_path)
    %%% fig 1: footprints
    fig1 = figure();

    hold on
    axis equal

    % box on
    % xlabel('$x$ [m]', 'Interpreter', 'LaTex');
    % ylabel('$y$ [m]', 'Interpreter', 'LaTex');
    axis off

    rwth_color = rwth_color_order();
    xlim([1.65 3.35]);
    ylim([1.1 2.5]);
    daspect([1 1 1])

    tick_now = 1;

    options = results{1, 1}.scenario;
    scenario = results{1, 1}.scenario;

    nVehs = options.amount;

    plot_lanelets(scenario.road_raw_data.lanelet, options.scenario_type);

    for v = 1:nVehs
        visualized_steps_num = [12, 5];

        for k = 2:2:visualized_steps_num(i_result)
            veh = scenario.vehicles(v);
            pos_step = results{i_result, v}.trajectory_predictions{v, k};
            x = pos_step(tick_now, :);
            vehiclePolygon = transformed_rectangle(x(1), x(2), x(3), veh.Length, veh.Width);

            if k == 8
                patch(vehiclePolygon(1, :) ...
                    , vehiclePolygon(2, :) ...
                    , rwth_color(5, :) ...
                    , 'LineWidth', 0.2 ...
                    , 'FaceAlpha', 0.8 ...
                    , 'EdgeColor', 'k' ...
                );
            else
                patch(vehiclePolygon(1, :) ...
                    , vehiclePolygon(2, :) ...
                    , rwth_color(v, :) ...
                    , 'LineWidth', 0.2 ...
                    , 'FaceAlpha', 1 - k / (max(visualized_steps_num) * 1.3) ...
                    , 'EdgeColor', 'k' ...
                );
            end

        end

    end

    % export fig
    set_figure_properties(fig1, ExportFigConfig.paper(paperwidth = 4.2))
    approaches = {'reachable_set', 'previous_plan'};
    filepath = fullfile(results_folder_path, ['eval-' approaches{i_result} '-footprints.pdf']); % full path of the fig
    export_fig(fig1, filepath);
    disp(['A figure was saved under ' filepath '.'])
    close(fig1);
end

function plot_viewpoint_reachable_set(i_result, results, results_folder_path)
    %%% fig 2: Viewpoint of vehicle 3 at k = 5
    fig2 = figure();

    hold on
    box on
    axis equal

    % xlabel('$x$ [m]', 'Interpreter', 'LaTex');
    % ylabel('$y$ [m]', 'Interpreter', 'LaTex');
    axis off

    xmin = 2.1; xmax = 3.2;
    ymin = 1.35; ymax = 2.35;

    xlim([xmin, xmax])
    ylim([ymin, ymax])
    daspect([1 1 1])

    options = results{i_result, 1}.options;
    scenario = results{i_result, 1}.scenario;
    nVehs = options.amount;
    rwth_color = rwth_color_order();
    tick_now = 1;
    step_idx = 5;

    plot_lanelets(scenario.road_raw_data.lanelet, options.scenario_type);
    ego_vehs = 3;
    other_vehs = setdiff(1:nVehs, ego_vehs);
    % plot reachable set
    for v = other_vehs
        reachable_sets = results{i_result, v}.iteration_data{step_idx}.reachable_sets(v, :);
        reachable_sets_array = cellfun(@(c) {[c.Vertices(:, 1)', c.Vertices(1, 1)'; c.Vertices(:, 2)', c.Vertices(1, 2)']}, reachable_sets);
        color = rwth_color(v, :);
        plot_cell_arrays(reachable_sets_array, color, true)
    end

    % plot predicted occupied areas of the ego vehicle
    trajectory_predictions = results{i_result, ego_vehs}.trajectory_predictions{ego_vehs, step_idx};
    is_one_step_shifted = false;
    plot_predicted_occupancy(trajectory_predictions, options, scenario, rwth_color(ego_vehs, :), scenario.mpa.trims_stop, is_one_step_shifted) % FIXME scenario.mpa

    % plot vehicle positions
    for v = 1:nVehs
        veh = scenario.vehicles(v);
        pos_step = results{i_result, v}.trajectory_predictions{v, step_idx};
        x = pos_step(tick_now, :);
        vehiclePolygon = transformed_rectangle(x(1), x(2), x(3), veh.Length, veh.Width);
        patch(vehiclePolygon(1, :) ...
            , vehiclePolygon(2, :) ...
            , rwth_color(v, :) ...
            , 'LineWidth', 0.2 ...
            , 'FaceAlpha', 0.90 ...
        );

        % print vehicle ID
        radius = veh.Width * 0.95/2;
        rectangle('Position', [x(1) - radius, x(2) - radius, 2 * radius, 2 * radius], 'Curvature', [1, 1], ...
            'FaceColor', [1, 1, 1, 0.75], 'LineStyle', 'none', 'LineWidth', 1, 'Tag', 'circle');
        text(x(1), x(2), num2str(v), 'FontSize', 7, 'LineWidth', 1, 'Color', 'black', 'HorizontalAlignment', 'center');
    end

    % export fig 2
    set_figure_properties(fig2, ExportFigConfig.paper(paperwidth = 4.2))
    approaches = {'reachable_set', 'previous_plan'};
    filepath = fullfile(results_folder_path, ['eval-' approaches{i_result} '-viewpoint.pdf']); % full path of the fig
    export_fig(fig2, filepath);
    disp(['A figure was saved under ' filepath '.'])
    close(fig2);
end

function plot_viewpoint_previous_occupancy(i_result, results, results_folder_path)
    %%% fig 2: Viewpoint of vehicle 3 at k = 5
    fig2 = figure();

    hold on
    axis equal

    % xlabel('$x$ [m]', 'Interpreter', 'LaTex');
    % ylabel('$y$ [m]', 'Interpreter', 'LaTex');
    axis off

    xmin = 2.1; xmax = 3.2;
    ymin = 1.35; ymax = 2.35;

    xlim([xmin, xmax])
    ylim([ymin, ymax])

    options = results{i_result, 1}.options;
    scenario = results{i_result, 1}.scenario;
    nVehs = options.amount;
    rwth_color = rwth_color_order();
    tick_now = 1;
    step_idx = 5;

    plot_lanelets(scenario.road_raw_data.lanelet, options.scenario_type);

    ego_vehs = 3;
    other_vehs = setdiff(1:nVehs, ego_vehs);

    % plot one-step-shifted previous plans
    for v = other_vehs
        trajectory_predictions = results{i_result, v}.trajectory_predictions{v, step_idx - 1};
        is_one_step_shifted = true;
        plot_predicted_occupancy(trajectory_predictions, options, scenario, rwth_color(v, :), scenario.mpa.trims_stop, is_one_step_shifted) % FIXME scenario.mpa
    end

    % plot predicted occupied areas of the ego vehicle
    trajectory_predictions = results{i_result, ego_vehs}.trajectory_predictions{ego_vehs, step_idx};
    is_one_step_shifted = false;
    plot_predicted_occupancy(trajectory_predictions, options, scenario, rwth_color(ego_vehs, :), scenario.mpa.trims_stop, is_one_step_shifted) % FIXME scenario.mpa

    % plot vehicle positions
    for v = 1:nVehs
        veh = scenario.vehicles(v);
        pos_step = results{i_result, v}.trajectory_predictions{v, step_idx};
        x = pos_step(tick_now, :);
        vehiclePolygon = transformed_rectangle(x(1), x(2), x(3), veh.Length, veh.Width);
        patch(vehiclePolygon(1, :) ...
            , vehiclePolygon(2, :) ...
            , rwth_color(v, :) ...
            , 'LineWidth', 0.2 ...
            , 'FaceAlpha', 0.90 ...
        );

        % print vehicle ID
        radius = veh.Width * 0.95/2;
        rectangle('Position', [x(1) - radius, x(2) - radius, 2 * radius, 2 * radius], 'Curvature', [1, 1], ...
            'FaceColor', [1, 1, 1, 0.75], 'LineStyle', 'none', 'LineWidth', 1, 'Tag', 'circle');
        text(x(1), x(2), num2str(v), 'FontSize', 7, 'LineWidth', 1, 'Color', 'black', 'HorizontalAlignment', 'center');
    end

    % export fig 2
    set_figure_properties(fig2, ExportFigConfig.paper(paperwidth = 4.2))
    approaches = {'reachable_set', 'previous_plan'};
    filepath = fullfile(results_folder_path, ['eval-' approaches{i_result} '-viewpoint.pdf']); % full path of the fig
    export_fig(fig2, filepath);
    disp(['A figure was saved under ' filepath '.'])
    close(fig2);
end

function plot_actual_plans(i_result, results, results_folder_path)
    %%% fig 3: Actual plans at k = 5
    fig3 = figure();
    % xlabel('$x$ [m]', 'Interpreter', 'LaTex');
    % ylabel('$y$ [m]', 'Interpreter', 'LaTex');
    axis off

    xmin = 2.1; xmax = 3.2;
    ymin = 1.35; ymax = 2.35;
    hold on
    box on
    axis equal
    xlim([xmin, xmax])
    ylim([ymin, ymax])
    daspect([1 1 1])

    options = results{i_result, 1}.options;
    scenario = results{i_result, 1}.scenario;
    nVehs = options.amount;
    rwth_color = rwth_color_order();
    tick_now = 1;
    step_idx = 5;

    plot_lanelets(scenario.road_raw_data.lanelet, options.scenario_type);

    % plot predicted occupied areas of all vehicle
    for v = 1:nVehs
        trajectory_predictions = results{i_result, v}.trajectory_predictions{v, step_idx};
        is_one_step_shifted = false;
        plot_predicted_occupancy(trajectory_predictions, options, scenario, rwth_color(v, :), scenario.mpa.trims_stop, is_one_step_shifted) % FIXME scenario.mpa
    end

    % plot vehicle positions
    for v = 1:nVehs
        veh = scenario.vehicles(v);
        pos_step = results{i_result, v}.trajectory_predictions{v, step_idx};
        x = pos_step(tick_now, :);
        vehiclePolygon = transformed_rectangle(x(1), x(2), x(3), veh.Length, veh.Width);
        patch(vehiclePolygon(1, :) ...
            , vehiclePolygon(2, :) ...
            , rwth_color(v, :) ...
            , 'LineWidth', 0.2 ...
        );

        % print vehicle ID
        radius = veh.Width * 0.95/2;
        rectangle('Position', [x(1) - radius, x(2) - radius, 2 * radius, 2 * radius], 'Curvature', [1, 1], ...
            'FaceColor', [1, 1, 1, 0.75], 'LineStyle', 'none', 'LineWidth', 1, 'Tag', 'circle');
        text(x(1), x(2), num2str(v), 'FontSize', 7, 'LineWidth', 1, 'Color', 'black', 'HorizontalAlignment', 'center');
    end

    % export fig 3
    set_figure_properties(fig3, ExportFigConfig.paper(paperwidth = 4.2))
    approaches = {'reachable_set', 'previous_plan'};
    filepath = fullfile(results_folder_path, ['eval-' approaches{i_result} '-actual.pdf']); % full path of the fig
    export_fig(fig3, filepath);
    close(fig3);
end
