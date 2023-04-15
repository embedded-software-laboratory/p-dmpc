function eval_parallel_computation_prediction_inconsistency()
%EVAL_PARALLEL_COMPUTATION_PREDICTION_INCONSISTENCY Evaluate of the strategy dealing with the prediction inconsistency problem in parallel
%computation paper
    disp('--------Prepare simulation data--------')
    isDealPredictionInconsistency = [true,false];
    % prepare simulation options
    options = Config();
    options.environment = Environment.Simulation;
    options.customResultName = '';
    options.scenario_name = 'Commonroad';
    options.trim_set = 9;
    options.Hp = 5;
    
    options.T_end = 4;
    options.dt = 0.2;
    options.max_num_CLs = 1;
    options.priority = 'STAC_priority';
    options.isPB = true;
    options.isParl = true;
    options.isAllowInheritROW = false;
    options.isSaveResult = true;
    options.isSaveResultReduced = false;
    options.options_plot_online = OptionsPlotOnline();
    options.is_eval = false;
    options.strategy_consider_veh_without_ROW = '1';
    options.strategy_enter_lanelet_crossing_area = '1';
    options.is_free_flow = false;
    
    options.reference_path.lanelets_index = {[95,69,64,62,75,55,53],[76,24,13,15,3,5],[12,73,92,94,100,101]};
    options.reference_path.start_point = [5,1,11];
    options.amount = length(options.reference_path.lanelets_index);
    options.veh_ids = 1:options.amount;
    results = cell(length(isDealPredictionInconsistency),options.amount);
    for i = 1:length(isDealPredictionInconsistency)
        options.isDealPredictionInconsistency = isDealPredictionInconsistency(i);
        if options.isDealPredictionInconsistency
            options.is_allow_collisions = false;
        else
            % if the prediction inconsistency problem is not addressed, collisions will be allowed, meaning that simulation will continue even if collisions occur
            options.is_allow_collisions = true;  
        end
    
        for veh_id = 1:options.amount
            full_path = FileNameConstructor.get_results_full_path(options,veh_id);
            if isfile(full_path)
                disp('File already exists.')
            else
                % run simulation
                [~,~] = main(options);
            end
            load(full_path,'result');
            results{i,veh_id} = result;
        end
    end
    disp('--------Simulation data prepared--------')

    % get the path of the target folder to store figures 
    [self_path,~,~] = fileparts(mfilename('fullpath')); % get the path of the current file
    idcs = strfind(self_path,filesep); % find all positions of '/'
    main_folder = self_path(1:idcs(end-1)-1); % two folders up, i.e., to main folder
    results_folder_path = fullfile(main_folder,'results'); % results folder
    if ~isfolder(results_folder_path)
        % create target folder if not exist
        mkdir(results_folder_path)
    end

    rwth_colors = rwth_color_order();
    veh_colors = rwth_colors(1:options.amount,:); 
    x_lim = [1.6 3.4];
    y_lim = [1.0 2.5];
    disp('--------Plot--------')
    plot_prediction_inconsistency_addressed(results,results_folder_path,veh_colors,x_lim,y_lim)
    plot_prediction_inconsistency_not_addressed(results,results_folder_path,veh_colors,x_lim,y_lim)
    disp('--------Plotted--------')

    % export video
    answer = questdlg('Would you like to export videos (may take several minutes)?', ...
	    'Export videos?', ...
	    'Yes','No','No');
    if strcmp(answer,'Yes')
        disp('--------Preparing simulation data for video--------')
        % combine distuibuted simulation data
        results_combined = combine_distributed_results(results);
        disp('--------Exporting videos--------')
        export_videos(results_combined,veh_colors,x_lim,y_lim,results_folder_path)
        disp('--------Videos exported--------')
    end
end

%% subfunction 1
function plot_prediction_inconsistency_addressed(results,results_folder_path,veh_colors,x_lim,y_lim)
    close all
    %%% fig 1: footprints
    fig1 = figure();

    hold on
    box on
    axis equal

    xlabel('$x$ [m]', 'Interpreter', 'LaTex');
    ylabel('$y$ [m]', 'Interpreter', 'LaTex');

    xlim(x_lim);
    ylim(y_lim);
    daspect([1 1 1])

    tick_now = 1;
    step_idx = 5;  
    i = 1;
    scenario = results{1,1}.scenario;
    
    nVehs = scenario.options.amount;

    plot_lanelets(scenario.road_raw_data.lanelet,scenario.options.scenario_name);
    for v = 1:nVehs
        visualized_steps_num = 13;
        for k = 1:2:visualized_steps_num
            veh = scenario.vehicles(v);
            pos_step = results{i,v}.trajectory_predictions{v,k};
            x = pos_step(tick_now,:);
            vehiclePolygon = transformedRectangle(x(1),x(2),x(3), veh.Length,veh.Width);
            
            if k == 9
                patch(   vehiclePolygon(1,:)...
                        ,vehiclePolygon(2,:)...
                        ,veh_colors(v,:)...
                        ,'LineWidth',0.2 ...
                        ,'FaceAlpha',1-k/(visualized_steps_num+2) ...
                        ,'EdgeColor','r'...
                );
            else
                patch(   vehiclePolygon(1,:)...
                        ,vehiclePolygon(2,:)...
                        ,veh_colors(v,:)...
                        ,'LineWidth',0.2 ...
                        ,'FaceAlpha',1-k/(visualized_steps_num+2) ...
                        ,'EdgeColor','k'...
                );
            end

            % print vehicle ID
            if k == 1
                radius = veh.Width * 0.95/2;
                rectangle('Position', [x(1) - radius, x(2) - radius, 2 * radius, 2 * radius], 'Curvature', [1, 1], ...
                    'FaceColor', [1, 1, 1, 0.75], 'LineStyle', 'none', 'LineWidth', 1, 'Tag', 'circle');
                text(x(1), x(2), num2str(v), 'FontSize', 5, 'LineWidth', 1, 'Color', 'black', 'HorizontalAlignment', 'center');
            end
        end
    end

    % export fig
    export_config = ExportFigConfig.paper;
    export_config.paperwidth = 5;
    set_figure_properties(fig1, export_config)
    filepath = fullfile(results_folder_path,'evalPaperConsiderReachableSetsA.pdf'); % full path of the fig
    export_fig(fig1, filepath);
    disp(['A figure was saved under ' filepath '.'])
    close(fig1);

    %     xlabel('(a) Footprints.')

    
    %%% fig 2: Viewpoint of vehicle 3 at k = 5
    fig2 = figure();

    hold on
    box on
    axis equal

    xlabel('$x$ [m]', 'Interpreter', 'LaTex');
    ylabel('$y$ [m]', 'Interpreter', 'LaTex');

    xmin = 2.1; xmax = 3.2;
    ymin = 1.35; ymax = 2.35;

    xlim([xmin,xmax])
    ylim([ymin,ymax])
    daspect([1 1 1])
    
    plot_lanelets(scenario.road_raw_data.lanelet,scenario.options.scenario_name);
    
    ego_vehs = 3;
    other_vehs = setdiff(1:nVehs,ego_vehs);
    % plot reachable set
    for v=other_vehs
        reachable_sets = results{i,v}.iteration_structs{step_idx}.reachable_sets(v,:);
        reachable_sets_array = cellfun(@(c) {[c.Vertices(:,1)',c.Vertices(1,1)';c.Vertices(:,2)',c.Vertices(1,2)']}, reachable_sets); 
        color = veh_colors(v,:);
        plot_cell_arrays(reachable_sets_array,color,true)
    end
    % plot predicted occupied areas of the ego vehicle
    trajectory_predictions = results{i,ego_vehs}.trajectory_predictions{ego_vehs, step_idx};
    is_one_step_shifted = false;
    plot_predicted_occupancy(trajectory_predictions,scenario,veh_colors(ego_vehs,:),scenario.mpa.trims_stop,is_one_step_shifted)

    % plot vehicle positions
    for v=1:nVehs
        veh = scenario.vehicles(v);
        pos_step = results{i,v}.trajectory_predictions{v,step_idx};
        x = pos_step(tick_now,:);
        vehiclePolygon = transformedRectangle(x(1),x(2),x(3), veh.Length,veh.Width);
        patch(   vehiclePolygon(1,:)...
                ,vehiclePolygon(2,:)...
                ,veh_colors(v,:)...
                ,'LineWidth',0.2 ...
                ,'FaceAlpha',0.90 ...
        );

        % print vehicle ID
        radius = veh.Width * 0.95/2;
        rectangle('Position', [x(1) - radius, x(2) - radius, 2 * radius, 2 * radius], 'Curvature', [1, 1], ...
            'FaceColor', [1, 1, 1, 0.75], 'LineStyle', 'none', 'LineWidth', 1, 'Tag', 'circle');
        text(x(1), x(2), num2str(v), 'FontSize', 7, 'LineWidth', 1, 'Color', 'black', 'HorizontalAlignment', 'center');
    end

    % export fig 2
    export_config = ExportFigConfig.paper;
    export_config.paperwidth = 5;
    set_figure_properties(fig2, export_config)
    filepath = fullfile(results_folder_path, 'evalPaperConsiderReachableSetsB.pdf');
    export_fig(fig2, filepath);
    disp(['A figure was saved under ' filepath '.'])
    close(fig2);
%     xlabel('(b) Viewpoint of vehicle 3 at {\it k} = 5.')
    
    %%% fig 3: Actual plans at k = 5
    fig3 = figure();
    xlabel('$x$ [m]', 'Interpreter', 'LaTex');
    ylabel('$y$ [m]', 'Interpreter', 'LaTex');

    xmin = 2.1; xmax = 3.2;
    ymin = 1.35; ymax = 2.35;
    hold on
    box on
    axis equal
    xlim([xmin,xmax])
    ylim([ymin,ymax])
    daspect([1 1 1])
    
    plot_lanelets(scenario.road_raw_data.lanelet,scenario.options.scenario_name);
    
    % plot predicted occupied areas of all vehicle
    for v = 1:nVehs    
        trajectory_predictions = results{i,v}.trajectory_predictions{v, step_idx};
        is_one_step_shifted = false;
        plot_predicted_occupancy(trajectory_predictions,scenario,veh_colors(v,:),scenario.mpa.trims_stop,is_one_step_shifted)
    end

    % plot vehicle positions
    for v = 1:nVehs
        veh = scenario.vehicles(v);
        pos_step = results{i,v}.trajectory_predictions{v,step_idx};
        x = pos_step(tick_now,:);
        vehiclePolygon = transformedRectangle(x(1),x(2),x(3), veh.Length,veh.Width);
        patch(   vehiclePolygon(1,:)...
                ,vehiclePolygon(2,:)...
                ,veh_colors(v,:)...
                ,'LineWidth',0.2 ...
                ,'FaceAlpha',0.90 ...
        );

        % print vehicle ID
        rectangle('Position', [x(1) - radius, x(2) - radius, 2 * radius, 2 * radius], 'Curvature', [1, 1], ...
            'FaceColor', [1, 1, 1, 0.75], 'LineStyle', 'none', 'LineWidth', 1, 'Tag', 'circle');
        text(x(1), x(2), num2str(v), 'FontSize', 7, 'LineWidth', 1, 'Color', 'black', 'HorizontalAlignment', 'center');
    end

    % export fig 3
    export_config = ExportFigConfig.paper;
    export_config.paperwidth = 5;
    set_figure_properties(fig3, export_config)
    filepath = fullfile(results_folder_path, 'evalPaperConsiderReachableSetsC.pdf');
    export_fig(fig3, filepath);
    disp(['A figure was saved under ' filepath '.'])
    close(fig3);
%     xlabel('(c) Actual plans at {\it k} = 5.')
end

%% subfunction 2
function plot_prediction_inconsistency_not_addressed(results,results_folder_path,veh_colors,x_lim,y_lim)
    close all
    %%% fig 1: footprints
    fig1 = figure();

    hold on
    box on
    axis equal

    xlabel('$x$ [m]', 'Interpreter', 'LaTex');
    ylabel('$y$ [m]', 'Interpreter', 'LaTex');

    xlim(x_lim);
    ylim(y_lim);
    daspect([1 1 1])

    tick_now = 1;
    step_idx = 5;  
    i = 2;
    scenario = results{1,1}.scenario;
    
    nVehs = scenario.options.amount;

    plot_lanelets(scenario.road_raw_data.lanelet,scenario.options.scenario_name);
    for v = 1:nVehs
        visualized_steps_num = 13;
        for k = 1:2:visualized_steps_num
            veh = scenario.vehicles(v);
            pos_step = results{i,v}.trajectory_predictions{v,k};
            x = pos_step(tick_now,:);
            vehiclePolygon = transformedRectangle(x(1),x(2),x(3), veh.Length,veh.Width);

            if k == 9
                % highlight the timestep when collision occurs
                patch(   vehiclePolygon(1,:)...
                        ,vehiclePolygon(2,:)...
                        ,veh_colors(v,:)...
                        ,'LineWidth',0.2 ...
                        ,'FaceAlpha',1-k/(visualized_steps_num+2) ...
                        ,'EdgeColor','r'...
                );
            else
                patch(   vehiclePolygon(1,:)...
                        ,vehiclePolygon(2,:)...
                        ,veh_colors(v,:)...
                        ,'LineWidth',0.2 ...
                        ,'FaceAlpha',1-k/(visualized_steps_num+2) ...
                        ,'EdgeColor','k'...
                );
            end
            % print vehicle ID
            if k==1
                veh = scenario.vehicles(v);
                radius = veh.Width * 0.95/2;
                rectangle('Position', [x(1) - radius, x(2) - radius, 2 * radius, 2 * radius], 'Curvature', [1, 1], ...
                    'FaceColor', [1, 1, 1, 0.75], 'LineStyle', 'none', 'LineWidth', 1, 'Tag', 'circle');
                text(x(1), x(2), num2str(v), 'FontSize', 5, 'LineWidth', 1, 'Color', 'black', 'HorizontalAlignment', 'center');
            end
        end
    end

    % export fig
    export_config = ExportFigConfig.paper;
    export_config.paperwidth = 5;
    set_figure_properties(fig1, export_config)
    filepath = fullfile(results_folder_path, 'evalPaperConsiderPreviousPlansA.pdf');
    export_fig(fig1, filepath);
    disp(['A figure was saved under ' filepath '.'])
    close(fig1);
%     xlabel('(a) Footprints.')
    

    %%% fig 2: Viewpoint of vehicle 3 at k = 5
    fig2 = figure();

    hold on
    box on
    axis equal

    xlabel('$x$ [m]', 'Interpreter', 'LaTex');
    ylabel('$y$ [m]', 'Interpreter', 'LaTex');

    xmin = 2.1; xmax = 3.2;
    ymin = 1.35; ymax = 2.35;

    xlim([xmin,xmax])
    ylim([ymin,ymax])
    daspect([1 1 1])
    
    plot_lanelets(scenario.road_raw_data.lanelet,scenario.options.scenario_name);
    
    ego_vehs = 3;
    other_vehs = setdiff(1:nVehs,ego_vehs);

    % plot one-step-shifted previous plans
    for v = other_vehs
        trajectory_predictions = results{i,v}.trajectory_predictions{v, step_idx-1};
        is_one_step_shifted = true;
        plot_predicted_occupancy(trajectory_predictions,scenario,veh_colors(v,:),scenario.mpa.trims_stop,is_one_step_shifted)
    end

    % plot predicted occupied areas of the ego vehicle
    trajectory_predictions = results{i,ego_vehs}.trajectory_predictions{ego_vehs, step_idx};
    is_one_step_shifted = false;
    plot_predicted_occupancy(trajectory_predictions,scenario,veh_colors(ego_vehs,:),scenario.mpa.trims_stop,is_one_step_shifted)

    % plot vehicle positions
    for v=1:nVehs
        veh = scenario.vehicles(v);
        pos_step = results{i,v}.trajectory_predictions{v,step_idx};
        x = pos_step(tick_now,:);
        vehiclePolygon = transformedRectangle(x(1),x(2),x(3), veh.Length,veh.Width);
        patch(   vehiclePolygon(1,:)...
                ,vehiclePolygon(2,:)...
                ,veh_colors(v,:)...
                ,'LineWidth',0.2 ...
                ,'FaceAlpha',0.90 ...
        );

        % print vehicle ID
        radius = veh.Width * 0.95/2;
        rectangle('Position', [x(1) - radius, x(2) - radius, 2 * radius, 2 * radius], 'Curvature', [1, 1], ...
            'FaceColor', [1, 1, 1, 0.75], 'LineStyle', 'none', 'LineWidth', 1, 'Tag', 'circle');
        text(x(1), x(2), num2str(v), 'FontSize', 7, 'LineWidth', 1, 'Color', 'black', 'HorizontalAlignment', 'center');
    end
    % export fig 2
    export_config = ExportFigConfig.paper;
    export_config.paperwidth = 5;
    set_figure_properties(fig2, export_config)
    filepath = fullfile(results_folder_path, 'evalPaperConsiderPreviousPlansB.pdf');
    export_fig(fig2, filepath);
    disp(['A figure was saved under ' filepath '.'])
    close(fig2);
    % xlabel('(b) Viewpoint of vehicle 3 at {\it k} = 5.')

    %%% fig 3: Actual plans at k = 5
    fig3 = figure();
    xlabel('$x$ [m]', 'Interpreter', 'LaTex');
    ylabel('$y$ [m]', 'Interpreter', 'LaTex');

    xmin = 2.1; xmax = 3.2;
    ymin = 1.35; ymax = 2.35;
    hold on
    box on
    axis equal
    xlim([xmin,xmax])
    ylim([ymin,ymax])
    daspect([1 1 1])
    
    plot_lanelets(scenario.road_raw_data.lanelet,scenario.options.scenario_name);
    
    % plot predicted occupied areas of all vehicle
    for v = 1:nVehs    
        trajectory_predictions = results{i,v}.trajectory_predictions{v, step_idx};
        is_one_step_shifted = false;
        plot_predicted_occupancy(trajectory_predictions,scenario,veh_colors(v,:),scenario.mpa.trims_stop,is_one_step_shifted)
    end

    % plot vehicle positions
    for v = 1:nVehs
        veh = scenario.vehicles(v);
        pos_step = results{i,v}.trajectory_predictions{v,step_idx};
        x = pos_step(tick_now,:);
        vehiclePolygon = transformedRectangle(x(1),x(2),x(3), veh.Length,veh.Width);
        patch(   vehiclePolygon(1,:)...
                ,vehiclePolygon(2,:)...
                ,veh_colors(v,:)...
                ,'LineWidth',0.2 ...
        );

        % print vehicle ID
        rectangle('Position', [x(1) - radius, x(2) - radius, 2 * radius, 2 * radius], 'Curvature', [1, 1], ...
            'FaceColor', [1, 1, 1, 0.75], 'LineStyle', 'none', 'LineWidth', 1, 'Tag', 'circle');
        text(x(1), x(2), num2str(v), 'FontSize', 7, 'LineWidth', 1, 'Color', 'black', 'HorizontalAlignment', 'center');
    end
    % export fig 3
    export_config = ExportFigConfig.paper;
    export_config.paperwidth = 5;
    set_figure_properties(fig3, export_config)
    filepath = fullfile(results_folder_path, 'evalPaperConsiderPreviousPlansC.pdf');
    export_fig(fig3, filepath);
    close(fig3);
    % xlabel('(c) Actual plans at {\it k} = 5.')

end

function results_combined = combine_distributed_results(results)
    results_combined = cell(size(results,1),1);
    for i = 1:size(results,1)
        result = results{i,1};
        for j = 2:size(results,2)
            trajectory_predictions = {results{i,j}.trajectory_predictions{j,:}};
            [result.trajectory_predictions{j,:}] = trajectory_predictions{:};
            for k = 1:length(results{i,j}.iteration_structs)
                result.iteration_structs{k}.referenceTrajectoryPoints(j,:,:) = results{i,j}.iteration_structs{k}.referenceTrajectoryPoints(j,:,:);
                result.iteration_structs{k}.referenceTrajectoryIndex(j,:) = results{i,j}.iteration_structs{k}.referenceTrajectoryIndex(j,:);
                result.iteration_structs{k}.v_ref(j,:) = results{i,j}.iteration_structs{k}.v_ref(j,:);
            end
        end
        results_combined{i} = result;
    end
end

% export videos
function export_videos(results,veh_colors,x_lim,y_lim,results_folder_path)
    for i = 1:numel(results)
        result = results{i};
    
        result.scenario.options.options_plot_online.is_custom_colors = true;
        result.scenario.options.options_plot_online.custom_colors = veh_colors;
        result.scenario.options.plot_limits(1,:) = x_lim;
        result.scenario.options.plot_limits(2,:) = y_lim;
        result.scenario.options.options_plot_online.plot_xy_labels = false;
        result.scenario.options.options_plot_online.plot_xy_ticks = false;
        result.scenario.options.options_plot_online.is_dynamic_colors = false;


        result.scenario.options.options_plot_online.plot_scenario_name = false;
        result.scenario.options.options_plot_online.plot_timesteps = true;


        result.scenario.options.options_plot_online.is_video_mode = true;
        result.scenario.options.options_plot_online.plot_coupling = true;
        result.scenario.options.options_plot_online.plot_weight = false;
        result.scenario.options.options_plot_online.plot_priority = false;

        if i == 1
            % plot the reachable sets of vehicles 1 and 2
            result.scenario.options.options_plot_online.plot_reachable_sets = true;
            result.scenario.options.options_plot_online.vehicles_reachable_sets = [1,2];
            result.scenario.options.options_plot_online.plot_predicted_occupancy = true;
            result.scenario.options.options_plot_online.vehicles_predicted_occupancy = [3];
            exportVideo(result,frame=30,name=fullfile(results_folder_path,'(a) consider reachable sets.mp4'))
            % plot the actual plans of vehicles 1 and 2
            result.scenario.options.options_plot_online.plot_reachable_sets = false;
            result.scenario.options.options_plot_online.plot_predicted_occupancy = true;
            result.scenario.options.options_plot_online.vehicles_predicted_occupancy = [1,2,3];
            exportVideo(result,frame=30,name=fullfile(results_folder_path,'(b) consider reachable sets.mp4'))
        else
            % plot the previously predicted occupancies of vehicles 1 and 2
            result.scenario.options.options_plot_online.plot_predicted_occupancy_previous = true;
            result.scenario.options.options_plot_online.vehicles_predicted_occupancy_previous = [1,2];
            result.scenario.options.options_plot_online.plot_predicted_occupancy = true;
            result.scenario.options.options_plot_online.vehicles_predicted_occupancy = [3];
            exportVideo(result,frame=30,name=fullfile(results_folder_path,'(a) consider previous plans'))
            % plot the actual plans of vehicles 1 and 2
            result.scenario.options.options_plot_online.plot_predicted_occupancy_previous = false;
            result.scenario.options.options_plot_online.plot_predicted_occupancy = true;
            result.scenario.options.options_plot_online.vehicles_predicted_occupancy = [1,2,3];
            exportVideo(result,frame=30,name=fullfile(results_folder_path,'(b) consider previous plans'))
        end
    end
end