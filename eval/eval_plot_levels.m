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

    %
    max_level = max(cellfun(@max,nLevels_by_pri));
    minVeh = min(nVeh_list);
    maxVeh = max(nVeh_list);
    % fill rows with zeros
    nLevels_by_pri_mat = [];
    max_entries = max(cellfun(@length,nLevels_by_pri));
    percent_steps_per_level = zeros(max_level, nPri);
    n_levels_min = cellfun(@min,nLevels_by_pri);
    n_levels_avg = cellfun(@mean,nLevels_by_pri);
    n_levels_max = cellfun(@max,nLevels_by_pri);
    n_levels_min_veh = cellfun(@min,nLevels_by_veh_pri); 
    n_levels_avg_veh = cellfun(@mean,nLevels_by_veh_pri);
    n_levels_max_veh = cellfun(@max,nLevels_by_veh_pri);
    bar_data = [n_levels_avg, n_levels_max];
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
        'mean', ...
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
    markers = {'x', 's', 'o', 'd'};
    [ ~, nPri ] = size(n_levels_max_veh);
    for iPri = 1:nPri
        plot(nVeh_list,n_levels_avg_veh(:,iPri),'Marker',markers{iPri},'Color',"#4DBEEE")
        hold on
    end
    for iPri = 1:nPri
        plot(nVeh_list,n_levels_max_veh(:,iPri),'Marker',markers{iPri},'Color','r')
        hold on
    end
    yticks(1:max_level)
    xticks(minVeh:maxVeh)
    ylabel('$N_{\mathrm{CL}}$','Interpreter','latex');
    xlabel('$N_{\mathrm{V}}$','Interpreter','latex');
    legend( ...
        'mean: FCA', ...
        'mean: Random', ...
        'mean: Constant', ...
        'mean: Coloring', ...
        'max: FCA', ...
        'max: Random', ...
        'max: Constant', ...
        'max: Coloring', ...
        'Location','best' ...
    );

    % Export
    folder_path = FileNameConstructor.gen_results_folder_path( ...
        res{1,1,1}.scenario.options ...
    );
    filename = 'computation-levels-graph.pdf';
    set_figure_properties(figHandle,'preset','paper','paperheight_in',6)
    export_fig(figHandle, fullfile(folder_path,filename));
    close(figHandle);
end

