function eval_parallel_computation_CLs()
%EVAL_PARALLEL_COMPUTATION_CLS Evaluate different number of computation levels
    disp('--------Prepare simulation data--------')
    options = Config();
    options.environment = Environment.Simulation;
    options.customResultName = '';
    options.scenario_name = 'Commonroad';
    options.trim_set = 9;
    options.priority = 'STAC_priority';
    options.isPB = true;
    options.isParl = true;
    options.isAllowInheritROW = false;
    options.strategy_consider_veh_without_ROW = '3';
    options.strategy_enter_lanelet_crossing_area = '1';
    options.dt = 0.2;
    
    options.isSaveResult = true;
    options.visu = [false,false];
    options.Hp = 7;
    options.T_end = 4;
    options.amount = 20;
    options.isParl = false; % avoid to use distributed computation
        
    random_times = 5;
    CLs_s = [1,2,4,6,options.amount];
    e_CLs = cell(random_times,length(CLs_s));
    results = cell(length(CLs_s),random_times);
    n_simulations = numel(e_CLs);
    count = 0;
    
    random_seed = RandStream('mt19937ar');
    for i = 1:random_times
        options.random_idx = i;
        for j = 1:length(CLs_s)
            if i==1 && j==1
                options.isSaveResultReduced = false;
            else
                options.isSaveResultReduced = true;
            end
            options.max_num_CLs = CLs_s(j);
            options.veh_ids = sort(randsample(random_seed,1:40,options.amount),'ascend');
            
            full_path = FileNameConstructor.get_results_full_path(options, options.amount);
            
            if isfile(full_path)
                disp('File already exists.')
            else
                % run simulation
                [~,~] = main(options);
            end
            % data processing
            e_CLs{i,j} = EvaluationParl(options);
            load(full_path,'result')
            results{i,j} = result;
            
            % display progress
            count = count + 1;
            disp(['--------Progress ' num2str(count) '/' num2str(n_simulations) ': done--------'])
        end
    end

    % get free flow speed, i.e., the speed that vehicles could travel if they are not influenced by others
    % vehicles
    free_flow_speed = get_free_flow_speed(options.dt);
    disp('--------Free flow speed calculated--------')
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

    disp('--------Ploting--------')
    plot_different_num_CLs(e_CLs,free_flow_speed,CLs_s,results_folder_path)
    disp('--------Plotted--------')

    % export video
%     answer = questdlg('Would you like to export videos (may take several minutes)?', ...
% 	    'Export videos?', ...
% 	    'Yes','No','No');
%     if strcmp(answer,'Yes')
%         disp('--------Exporting videos--------')
%         export_videos(results)
%         disp('--------Videos exported--------')
%     end
end

%% plot average speed and the actual number of computation levels
function plot_different_num_CLs(e_CLs,free_flow_speed,CLs_s,results_folder_path)
    close all

    %%% fig 1: Average speed (normalized)
    fig1 = figure();

    hold on
    box on
    grid on 

    % maximum number of actual computation levels
    CLs_num_max = cellfun(@(c) c.CLs_num_max, e_CLs);
    
    X_string = string(CLs_s);
    X_string(1) = 'Parl.';
    X_string(2) = ['$N_{l,a}=$' num2str(CLs_s(2))];
    X_string(end) = 'Alr.';
    
    % plot average speed
    average_speed_s = cellfun(@(c) c.average_speed, e_CLs);
    average_speed_normalized = average_speed_s./free_flow_speed;

    boxplot(average_speed_normalized*100,'Colors','k','OutlierSize',4,'Labels',X_string);
    
    y_FFS = yline(free_flow_speed/free_flow_speed*100,'--b','LineWidth',0.6);
    
    legend(y_FFS,{'Free-flow'},'Location','south')
    ylabel('$\overline{v}_n\:[\%]$','Interpreter','latex')
    % xlabel('(a) Average speed (normalized).')
    xaxis = get(gca, 'XAxis');
    xaxis.TickLabelInterpreter = 'latex'; % latex for x-axis
    
    ylim([0 105])
    yticks(0:20:105)
    
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

    boxplot(CLs_num_max,'Colors','k','OutlierSize',4,'Labels',X_string);

    level_max = 12;
    ylim([0 level_max])
    yticks(0:2:level_max)
    % xlabel('(b) Actual number of computation levels.')
    ylabel('$N_{l}$','Interpreter','latex')
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
function export_videos(results)
    for i = 1:numel(results)
        result = results{i};
    
        result.scenario.options.optionsPlotOnline.isVideoMode = true;
        result.scenario.options.optionsPlotOnline.isShowCoupling = true;
        result.scenario.options.optionsPlotOnline.isShowPriority = true;
        
        % videoExportSetup.framerate = 30;
        exportVideo(result)
    end
end