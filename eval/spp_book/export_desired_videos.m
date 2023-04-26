function export_desired_videos(res)
    % Export videos of the desired scenarios
    % For each priority assignment algorithm, export a video of the scenario
    % showing the best and worst performance regarding deadlocks

    % iterate over scenarios
    % get best and worst scenario
    [best, worst] = find_best_worst_result(res);
    for r = [best, worst]
        % export videos
            % load result to get trajectory predictions
            results_full_path = FileNameConstructor.get_results_full_path( ...
                r.scenario.options ...
            );
            if isfile(results_full_path)
                r_full_loaded = load(results_full_path);
                r_full = r_full_loaded.result;
            else
                error('Experiment results required in "results" folder')
            end
            r_full.scenario.options.options_plot_online.plot_priority = true;
            r_full.scenario.options.options_plot_online.plot_coupling = true;
            export_video(r_full);
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
    best = [];
    while ( inVeh >= 1 ) && ( found == false )
        for iSce = 1:nSce
            for iPri = 1:nPri
                [~, standstill_free_time(iPri)] = compute_deadlock_free_runtime(res{inVeh,iPri,iSce});
            end
            if all(standstill_free_time == t_total)
                found = true;
                best = [res{inVeh,:,iSce}];
                break;
            end
        end
        inVeh = inVeh - 1;
    end

    % get worst performance
    % find scenario for which deadlock occured with fewest vehicles
    inVeh = 1;
    found = false;
    worst = [];
    while ( inVeh <= nVeh ) && ( found == false )
        for iSce = 1:nSce
            for iPri = 1:nPri
                [~, standstill_free_time(iPri)] = compute_deadlock_free_runtime(res{inVeh,iPri,iSce});
            end
            standstill_time = t_total - standstill_free_time;
            if all( standstill_time > 0 )
                found = true;
                worst = [res{inVeh,:,iSce}];
                break;
            end
        end
        inVeh = inVeh + 1;
    end

end