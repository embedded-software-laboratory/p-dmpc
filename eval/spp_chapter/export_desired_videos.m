function export_desired_videos(res)
    % Export videos of the desired scenarios
    % For each priority assignment algorithm, export a video of the scenario
    % showing the best and worst performance regarding deadlocks

    [ ~, nPri, ~ ] = size(res);
    for iPri = 1:nPri
        % iterate over scenarios
        % get best and worst scenario
        [best, worst] = find_best_worst_scenario(res,iPri);
        % export videos
        best.scenario.options.optionsPlotOnline.isShowPriority = true;
        best.scenario.options.optionsPlotOnline.isShowCoupling = true;
        worst.scenario.options.optionsPlotOnline.isShowPriority = true;
        worst.scenario.options.optionsPlotOnline.isShowCoupling = true;
        exportVideo(best);
        exportVideo(worst);
    end
end

function [best, worst] = find_best_worst_scenario(res,iPri)
    % Get the best and worst scenario regarding deadlock-free runtime
    % for the given priority assignment algorithm

    [ nVeh, ~, nSce ] = size(res);
    % assumes that all scenarios have the same total runtime
    t_total = res{1,1,1}.t_total;

    % get best performance
    % find scenario for which no deadlock occured with most vehicles
    for inVeh = nVeh:-1:1
        for iSce = 1:nSce
            deadlock_free_runtime = compute_deadlock_free_runtime(res{inVeh,iPri,iSce});
            if (deadlock_free_runtime == t_total)
                iSce_best = iSce;
                inVeh_best = inVeh;
                break;
            end
        end
    end

    % get worst performance
    % find scenario for which deadlock occured with fewest vehicles
    for inVeh = 1:nVeh
        for iSce = 1:nSce
            deadlock_free_runtime = compute_deadlock_free_runtime(res{inVeh,iPri,iSce});
            deadlocked_runtime = deadlock_free_runtime - t_total;
            if (deadlocked_runtime > 0)
                iSce_worst = iSce;
                inVeh_worst = inVeh;
                break;
            end
        end
    end

    best = res{inVeh_best,iPri,iSce_best};
    worst = res{inVeh_worst,iPri,iSce_worst};
end