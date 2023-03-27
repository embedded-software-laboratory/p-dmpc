function eval_parallel_computation_prediction_inconsistency()
%EVAL_PARALLEL_COMPUTATION_PREDICTION_INCONSISTENCY Evaluate of the strategy dealing with the prediction inconsistency problem in parallel
%computation paper
    disp('--------Prepare simulation data--------')
    isDealPredictionInconsistency = [true,false];
    % prepare simulation options
    options = Config();
    options.is_sim_lab = true;
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
    options.visu = [true,false];
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
                if exist('options','var') && exist('scenario','var')
                    [~,~] = main(options,scenario);
                else
                    [~,scenario] = main(options);
                end
            end
            load(full_path,'result');
            results{i,veh_id} = result;
        end
    end
    disp('--------Simulation data prepared--------')

    disp('--------Plot--------')
    plot_prediction_inconsistency_addressed(results)
    plot_prediction_inconsistency_not_addressed(results)
    disp('--------Plotted--------')
end

%% subfunction 1
function plot_prediction_inconsistency_addressed(results)
    close all
    %%% fig 1: footprints
    fig1 = figure();

    hold on
    box on
    axis equal

    xlabel('$x$ [m]', 'Interpreter', 'LaTex');
    ylabel('$y$ [m]', 'Interpreter', 'LaTex');

    rwth_color = rwth_color_order();
    xlim([1.6 4]);
    ylim([1.05 2.75]);
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
                        ,rwth_color(v,:)...
                        ,'LineWidth',0.2 ...
                        ,'FaceAlpha',1-k/(visualized_steps_num+2) ...
                        ,'EdgeColor','r'...
                );
            else
                patch(   vehiclePolygon(1,:)...
                        ,vehiclePolygon(2,:)...
                        ,rwth_color(v,:)...
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
    filepath = fullfile('results', 'evalPaperConsiderReachableSetsA.pdf');
    export_fig(fig1, filepath);
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
        color = rwth_color(v,:);
        plot_cell_arrays(reachable_sets_array,color,true)
    end
    % plot predicted occupied areas of the ego vehicle
    trajectory_predictions = results{i,ego_vehs}.trajectory_predictions{ego_vehs, step_idx};
    is_one_step_shifted = false;
    plot_predicted_occupancy(trajectory_predictions,scenario,rwth_color(ego_vehs,:),scenario.mpa.trims_stop,is_one_step_shifted)

    % plot vehicle positions
    for v=1:nVehs
        veh = scenario.vehicles(v);
        pos_step = results{i,v}.trajectory_predictions{v,step_idx};
        x = pos_step(tick_now,:);
        vehiclePolygon = transformedRectangle(x(1),x(2),x(3), veh.Length,veh.Width);
        patch(   vehiclePolygon(1,:)...
                ,vehiclePolygon(2,:)...
                ,rwth_color(v,:)...
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
    filepath = fullfile('results', 'evalPaperConsiderReachableSetsB.pdf');
    export_fig(fig2, filepath);
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
        plot_predicted_occupancy(trajectory_predictions,scenario,rwth_color(v,:),scenario.mpa.trims_stop,is_one_step_shifted)
    end

    % plot vehicle positions
    for v = 1:nVehs
        veh = scenario.vehicles(v);
        pos_step = results{i,v}.trajectory_predictions{v,step_idx};
        x = pos_step(tick_now,:);
        vehiclePolygon = transformedRectangle(x(1),x(2),x(3), veh.Length,veh.Width);
        patch(   vehiclePolygon(1,:)...
                ,vehiclePolygon(2,:)...
                ,rwth_color(v,:)...
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
    filepath = fullfile('results', 'evalPaperConsiderReachableSetsC.pdf');
    export_fig(fig3, filepath);
    close(fig3);
%     xlabel('(c) Actual plans at {\it k} = 5.')
end

%% subfunction 2
function plot_prediction_inconsistency_not_addressed(results)
    close all
    %%% fig 1: footprints
    fig1 = figure();

    hold on
    box on
    axis equal

    xlabel('$x$ [m]', 'Interpreter', 'LaTex');
    ylabel('$y$ [m]', 'Interpreter', 'LaTex');

    rwth_color = rwth_color_order();
    xlim([1.6 4]);
    ylim([1.05 2.75]);
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
                        ,rwth_color(v,:)...
                        ,'LineWidth',0.2 ...
                        ,'FaceAlpha',1-k/(visualized_steps_num+2) ...
                        ,'EdgeColor','r'...
                );
            else
                patch(   vehiclePolygon(1,:)...
                        ,vehiclePolygon(2,:)...
                        ,rwth_color(v,:)...
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
    filepath = fullfile('results', 'evalPaperConsiderPreviousPlansA.pdf');
    export_fig(fig1, filepath);
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
        plot_predicted_occupancy(trajectory_predictions,scenario,rwth_color(v,:),scenario.mpa.trims_stop,is_one_step_shifted)
    end

    % plot predicted occupied areas of the ego vehicle
    trajectory_predictions = results{i,ego_vehs}.trajectory_predictions{ego_vehs, step_idx};
    is_one_step_shifted = false;
    plot_predicted_occupancy(trajectory_predictions,scenario,rwth_color(ego_vehs,:),scenario.mpa.trims_stop,is_one_step_shifted)

    % plot vehicle positions
    for v=1:nVehs
        veh = scenario.vehicles(v);
        pos_step = results{i,v}.trajectory_predictions{v,step_idx};
        x = pos_step(tick_now,:);
        vehiclePolygon = transformedRectangle(x(1),x(2),x(3), veh.Length,veh.Width);
        patch(   vehiclePolygon(1,:)...
                ,vehiclePolygon(2,:)...
                ,rwth_color(v,:)...
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
    filepath = fullfile('results', 'evalPaperConsiderPreviousPlansB.pdf');
    export_fig(fig2, filepath);
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
        plot_predicted_occupancy(trajectory_predictions,scenario,rwth_color(v,:),scenario.mpa.trims_stop,is_one_step_shifted)
    end

    % plot vehicle positions
    for v = 1:nVehs
        veh = scenario.vehicles(v);
        pos_step = results{i,v}.trajectory_predictions{v,step_idx};
        x = pos_step(tick_now,:);
        vehiclePolygon = transformedRectangle(x(1),x(2),x(3), veh.Length,veh.Width);
        patch(   vehiclePolygon(1,:)...
                ,vehiclePolygon(2,:)...
                ,rwth_color(v,:)...
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
    filepath = fullfile('results', 'evalPaperConsiderPreviousPlansC.pdf');
    export_fig(fig3, filepath);
    close(fig3);
    % xlabel('(c) Actual plans at {\it k} = 5.')

end

%% helper function
function p = plot_cell_arrays(cells,color,isFill)
% Plot shapes contained in a cell array
%     CM = jet(Hp); % colormap
    hold on
    if nargin==1
        isFill = false;
    end
    if isFill
        for j = size(cells,2):-1:1
            shape = cells{j};
            patch(shape(1,:),shape(2,:),color,'FaceAlpha',(1-j/(size(cells,2)+2))*0.5);
        end
    else
        CM = rwth_color_order(size(cells,2)+3);
        CM = CM(4:end,:);
    
        for j = 1:size(cells,2)
            shape = cells{j};
            p(j) = plot(shape(1,:),shape(2,:),'LineWidth',1,'Color',CM(j,:),'LineStyle','-.');
        end
    end
end

function plot_predicted_occupancy(trajectory_predictions,scenario,color,trims_stop,is_one_step_shifted)
% plot predicted occupied areas
    n_predictions = size(trajectory_predictions, 1);
    n_ticks = n_predictions / scenario.options.Hp;

    i = 0;
    for tick = n_predictions-n_ticks+1:-n_ticks:1
        if is_one_step_shifted && tick == 1
            % ignore the first timestep
            continue
        end
        i = i + 1;
        x = trajectory_predictions(tick, :);
        trim1 = trajectory_predictions(tick, 4);
        if i == 1
            assert(length(trims_stop)==1)
            trim2 = trims_stop;
        else
            trim2 = trajectory_predictions(tick + n_ticks, 4);
        end
        area = scenario.mpa.maneuvers{trim1, trim2}.area;
        [area_x, area_y] = translate_global(x(3), x(1), x(2), area(1, :), area(2, :));
        area_poly = polyshape([area_x; area_y]');
        plot(area_poly, 'FaceColor', color, 'FaceAlpha', (i/(scenario.options.Hp+2))*0.5)
    end
end