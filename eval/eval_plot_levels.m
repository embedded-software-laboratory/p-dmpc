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
        '$p_{r}$', ...
        '$p_{\mathrm{const}}$', ...
        '$p_{\mathrm{color}}$', ...
        'Location','best' ...
    );
    xlabel('$n_{\mathrm{CL}}$','Interpreter','latex');
    ylabel('Occurences from all steps [\%]','Interpreter','latex');


    % Export
    folder_path = FileNameConstructor.gen_results_folder_path(res{1,1,1}.scenario.options);
    filename = 'computation-levels.pdf';
    set_figure_properties(figHandle,'paper',7)
    export_fig(figHandle, fullfile(folder_path,filename));
    close(figHandle);
end

