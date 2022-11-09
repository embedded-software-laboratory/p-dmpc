function export_desired_videos(res)
    % Export videos of the desired scenarios
    % For each priority assignment algorithm, export a video of the scenario
    % showing the best and worst performance regarding deadlocks

    [ ~, nPri, ~ ] = size(res);
    for iPri = 1:nPri
        % iterate over scenarios
        % get best and worst scenario
        [best, worst] = find_best_worst_result(res,iPri);
        for r_cell = [best, worst]
            % export videos
            for r = [r_cell{:}]
                % load result to get trajectory predictions
                results_full_path = FileNameConstructor.get_results_full_path(res.scenario.options);
                if isfile(results_full_path)
                    r_full_loaded = load(results_full_path);
                    r_full = r_full_loaded.result;
                else
                    error('Experiment results required in "results" folder')
                end
                r_full.scenario.options.optionsPlotOnline.isShowPriority = true;
                r_full.scenario.options.optionsPlotOnline.isShowCoupling = true;
                exportVideo(r_full);
            end
        end
    end
end

function [best, worst] = find_best_worst_result(res)
    % Get the best and worst scenario regarding deadlock-free runtime
    % for the given priority assignment algorithm

    [ nVeh, nPri, nSce ] = size(res);
    % assumes that all scenarios have the same total runtime
    t_total = res{1,1,1}.t_total;

    % get best performance
    % find scenario for which no deadlock occured with most vehicles
    inVeh = nVeh;
    found = false;
    standstill_free_time = zeros(nPri,1);
    while ( inVeh >= 1 ) && ( found == false )
        for iSce = 1:nSce
            for iPri = 1:nPri
                [~, standstill_free_time(iPri)] = compute_deadlock_free_runtime(res{inVeh,iPri,iSce});
            end
            if all(standstill_free_time == t_total)
                iSce_best = iSce;
                inVeh_best = inVeh;
                found = true;
                break;
            end
        end
        inVeh = inVeh - 1;
    end

    % get worst performance
    % find scenario for which deadlock occured with fewest vehicles
    inVeh = 1;
    found = false;
    while ( inVeh <= nVeh ) && ( found == false )
        for iSce = 1:nSce
            for iPri = 1:nPri
                [~, standstill_free_time(iPri)] = compute_deadlock_free_runtime(res{inVeh,iPri,iSce});
            end
            standstill_time = standstill_free_time - t_total;
            if all( standstill_time > 0 )
                iSce_worst = iSce;
                inVeh_worst = inVeh;
                found = true;
                break;
            end
        end
        inVeh = inVeh + 1;
    end

    best = res{inVeh_best,:,iSce_best};
    worst = res{inVeh_worst,:,iSce_worst};
end