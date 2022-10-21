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
    for iVeh = 1:nVeh
        for iPri = 1:nPri
            for iSce = 1:nSce
                result = res{iVeh,iPri,iSce};
                % get number of steps until deadlock
                [nSteps,~] = compute_deadlock_free_runtime(result);
                % get number of levels by max priority assigned
                result.nLevels = max(result.priority(:,1:nSteps));
                nLevels_by_veh_pri{iVeh,iPri} = [nLevels_by_veh_pri{iVeh,iPri} result.nLevels];
                nLevels_by_pri{iPri} = [nLevels_by_pri{iPri} result.nLevels];
            end
        end
    end

    %
    max_level = max(cellfun(@max,nLevels_by_pri));
    % fill rows with zeros
    nLevels_by_pri_mat = [];
    max_entries = max(cellfun(@length,nLevels_by_pri));
    percent_steps_per_level = zeros(max_level, nPri);
    n_levels_min = cellfun(@min,nLevels_by_pri);
    n_levels_avg = cellfun(@mean,nLevels_by_pri);
    n_levels_max = cellfun(@max,nLevels_by_pri);
    bar_data = [n_levels_min, n_levels_avg, n_levels_max];
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
        '$p_{\mathrm{const}}$'
    });


    lgd = legend( ...
        'min', ...
        'mean', ...
        'max' ...
    );
    xlim([1, max_level+1])


    % Export
    folder_path = FileNameConstructor.gen_results_folder_path( ...
        res{1,1,1}.scenario.options ...
    );
    filename = 'computation-levels-overview.pdf';
    set_figure_properties(figHandle,'preset','paper','paperheight_in',6)
    lgd.Position = [0.63 0.72 0.26 0.19];
    export_fig(figHandle, fullfile(folder_path,filename));
    close(figHandle);
end

