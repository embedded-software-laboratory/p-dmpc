function eval_plot_levels(res)
% EVAL_PLOT_LEVELS  Evaluate the computation levels of the result cell `res`.

    resstruct = [res{:}];
    runtimes = [resstruct.t_total];
    is_equal_runtime = all(runtimes == runtimes(1));
    if ~is_equal_runtime
        warning("different experiment durations");
    end

    [ nVeh, nPri, nSce ] = size(res);
    nLevels_by_veh_pri = cell(nVeh, nPri);
    nLevels_by_pri = cell(nPri,1);
    nVeh_list = zeros(1,nVeh);
    for iVeh = 1:nVeh
        nVeh_list(iVeh) = res{iVeh,1,1}.scenario.options.amount;
        for iPri = 1:nPri
            for iSce = 1:nSce
                result = res{iVeh,iPri,iSce};
        
                % get number of steps until deadlock
                [nSteps,~] = compute_deadlock_free_runtime(result);
        
                scenario_tmp = result.scenario;
        
                for iStep = nSteps:-1:1
                    % no adjacency given in 1-veh scenarios
                    if scenario_tmp.options.amount > 1
                        scenario_tmp.adjacency = scenario_tmp.adjacency(:,:,1:iStep);
                        scenario_tmp.semi_adjacency = scenario_tmp.semi_adjacency(:,:,1:iStep);
            
                        iter_tmp = result.iteration_structs{iStep};
                    
                        % assign priorities using different algorithms
                        [~, ~, ~, fca_prios] = FCA_priority().priority(scenario_tmp,iter_tmp);
                        [~, ~, random_prios] = random_priority().priority(scenario_tmp);
                        [~, ~, constant_prios] = constant_priority().priority(scenario_tmp);
                        [~, ~, coloring_prios] = coloring_priority().priority(scenario_tmp);
            
                        % get number of levels by max priority assigned
                        n_fca = max(fca_prios);
                        n_random = max(random_prios);
                        n_constant = max(constant_prios);
                        n_coloring = max(coloring_prios);
                    else
                        [n_fca, n_random, n_constant, n_coloring] = deal(1);
                    end

                    
                    nLevels_by_veh_pri{iVeh,1} = [nLevels_by_veh_pri{iVeh,1} n_fca];
                    nLevels_by_veh_pri{iVeh,2} = [nLevels_by_veh_pri{iVeh,2} n_random];
                    nLevels_by_veh_pri{iVeh,3} = [nLevels_by_veh_pri{iVeh,3} n_constant];
                    nLevels_by_veh_pri{iVeh,4} = [nLevels_by_veh_pri{iVeh,4} n_coloring];
        
                    nLevels_by_pri{1} = [nLevels_by_pri{1} n_fca];
                    nLevels_by_pri{2} = [nLevels_by_pri{2} n_random];
                    nLevels_by_pri{3} = [nLevels_by_pri{3} n_constant];
                    nLevels_by_pri{4} = [nLevels_by_pri{4} n_coloring];
                end
            end
        end
    end

    max_level = max(cellfun(@max,nLevels_by_pri));
    minVeh = min(nVeh_list);
    maxVeh = max(nVeh_list);
    % fill rows with zeros
    nLevels_by_pri_mat = [];
    max_entries = max(cellfun(@length,nLevels_by_pri));
    percent_steps_per_level = zeros(max_level, nPri);
    n_levels_min = cellfun(@min,nLevels_by_pri);
    n_levels_avg = cellfun(@mean,nLevels_by_pri);
    n_levels_med = cellfun(@median,nLevels_by_pri);
    n_levels_max = cellfun(@max,nLevels_by_pri);
    n_levels_min_veh = cellfun(@min,nLevels_by_veh_pri); 
    n_levels_avg_veh = cellfun(@mean,nLevels_by_veh_pri);
    n_levels_med_veh = cellfun(@median,nLevels_by_veh_pri);
    n_levels_max_veh = cellfun(@max,nLevels_by_veh_pri);
    bar_data = [n_levels_med, n_levels_max];
    for iPri = 1:nPri
        nSteps_per_level = histcounts(nLevels_by_pri{iPri},1:max_level+1);
        percent_steps_per_level(:,iPri) = nSteps_per_level/length(nLevels_by_pri{iPri});
        tmp = nLevels_by_pri{iPri};
        tmp(end+1:max_entries) = 0;
        nLevels_by_pri{iPri} = tmp;
        nLevels_by_pri_mat = [nLevels_by_pri_mat; nLevels_by_pri{iPri}]; %#ok<AGROW> 
    end
    % plot    
    figHandle = figure();
    barh(1:max_level, percent_steps_per_level);
    set(gca,'Ydir','reverse');
    legend( ...
        '$p_{\mathrm{fca}}$', ...
        '$p_{\mathrm{rand}}$', ...
        '$p_{\mathrm{const}}$', ...
        '$p_{\mathrm{color}}$', ...
        'Location','best' ...
    );
    xlabel('Occurences from all steps [\%]','Interpreter','latex');
    ylabel('$N_{\mathrm{CL}}$','Interpreter','latex');


    % Export
    folder_path = FileNameConstructor.gen_results_folder_path( ...
        res{1,1,1}.scenario.options ...
    );
    filename = 'computation-levels-detail.pdf';
    set_figure_properties(figHandle,'preset','paper','paperheight_in',7)
    export_fig(figHandle, fullfile(folder_path,filename));
    close(figHandle);

    % plot
    figHandle = figure();
    % sort data
    data_permutation = [1 4 2 3];
    bar_data = bar_data(data_permutation,:);
    barh(1:nPri, bar_data);
    set(gca,'Ydir','reverse');
    y_axis_handle = get(gca, 'YAxis');
    y_axis_handle.TickLabelInterpreter = 'latex';
    xlabel('$N_{\mathrm{CL}}$','Interpreter','latex');
    ylabel('Priority Assignment Function');
    yticklabels({ ...
        '$p_{\mathrm{fca}}$', ...
        '$p_{\mathrm{color}}$', ...
        '$p_{\mathrm{rand}}$', ...
        '$p_{\mathrm{const}}$', ...
    });


    legend( ...
        'median', ...
        'max', ...
        'Location','best' ...
    );
    xlim([0, max_level+1])
    xticks([2,4,6])


    % Export
    folder_path = FileNameConstructor.gen_results_folder_path( ...
        res{1,1,1}.scenario.options ...
    );
    filename = 'computation-levels-overview.pdf';
    set_figure_properties(figHandle,'preset','paper','paperheight_in',6)
    export_fig(figHandle, fullfile(folder_path,filename));
    close(figHandle);

    % plot
    figHandle = figure();
    markers = {'o', 'x', 'd', '+'
                '^', 'pentagram', '*', 's'};
    colors = ["#0072BD","#D95319","#7E2F8E","#77AC30"];
    [ ~, nPri ] = size(n_levels_max_veh);
    for iPri = 1:nPri
        plot(nVeh_list,n_levels_med_veh(:,iPri),'Marker',markers{1,iPri},'Color',colors(iPri),'LineStyle','none','MarkerSize',1)
        hold on
        plot(nVeh_list,n_levels_max_veh(:,iPri),'Marker',markers{2,iPri},'Color',colors(iPri),'LineStyle','none','MarkerSize',1)
    end
    yticks(1:max_level)
    xticks(minVeh:maxVeh)
    ylabel('$N_{\mathrm{CL}}$','Interpreter','latex');
    xlabel('$N_{\mathrm{V}}$','Interpreter','latex');
    legend( ...
        'median: FCA', ...
        'max: FCA', ...
        'median: Random', ...
        'max: Random', ...
        'median: Constant', ...
        'max: Constant', ...
        'median: Coloring', ...
        'max: Coloring', ...
        'Location','best' ...
    );

    % Export
    folder_path = FileNameConstructor.gen_results_folder_path( ...
        res{1,1,1}.scenario.options ...
    );
    filename = 'computation-levels-graph.pdf';
    % TODO paper size to guarantee legend does not cover graph
    set_figure_properties(figHandle,'preset','paper','paperheight_in',15,'paperwidth_in',15)
    export_fig(figHandle, fullfile(folder_path,filename));
    close(figHandle);

    % plot
    figHandle = figure();
    [ ~, nPri ] = size(n_levels_max_veh);
    bar(n_levels_max_veh(:,data_permutation))
    hold on
    bar(n_levels_med_veh(:,data_permutation))
    yticks(1:max_level)
    xticks(minVeh:maxVeh)
    ylabel('$N_{\mathrm{CL}}$','Interpreter','latex');
    xlabel('$N_{\mathrm{A}}$','Interpreter','latex');
    legend( ...
        'max: $p_{\mathrm{fca}}$', ...
        'max: $p_{\mathrm{color}}$', ...
        'max: $p_{\mathrm{rand}}$', ...
        'max: $p_{\mathrm{const}}$', ...
        'median: $p_{\mathrm{fca}}$', ...
        'median: $p_{\mathrm{color}}$', ...
        'median: $p_{\mathrm{rand}}$', ...
        'median: $p_{\mathrm{const}}$', ...
        'Location','best' ...
    );

    % Export
    folder_path = FileNameConstructor.gen_results_folder_path( ...
        res{1,1,1}.scenario.options ...
    );
    filename = 'computation-levels-bar-plot.pdf';
    % TODO paper size to guarantee legend does not cover graph
    set_figure_properties(figHandle,'preset','paper','paperheight_in',6,'paperwidth_in',12)
    % overwrite color order
    r100 = rwth_color_order;
    r50 = rwth_color_order_50;
    colororder(figHandle,[r50(1:nPri,:);r100(1:nPri,:)]);
    export_fig(figHandle, fullfile(folder_path,filename));
    close(figHandle);
end