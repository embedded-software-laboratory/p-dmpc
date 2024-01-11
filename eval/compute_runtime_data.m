function [data] = compute_runtime_data(res)
    % COMPUTE_RUNTIME_DATA  Calculate data needed to export plots for runtime evaluation

    data = struct();

    [nVeh, nPri, nSce] = size(res);

    x_values = zeros(1, nVeh);
    t_deadlock_free = zeros(nVeh, nSce, nPri);
    n_deadlock_free = zeros(nVeh, nSce, nPri);
    avg_speed_by_veh_pri = zeros(nVeh, nPri);

    for iVeh = 1:nVeh
        x_values(iVeh) = res{iVeh, 1, 1}.options.amount;

        for iPri = 1:nPri
            avg_speed_scenario = zeros(nSce, 1);

            for iSce = 1:nSce
                disp(['Scenario ', num2str(iVeh), ' / ', num2str(iPri), ' / ', num2str(iSce)])
                experiment_result = res{iVeh, iPri, iSce};
                [n_total_deadlock_free, t_total_deadlock_free] = ...
                    compute_deadlock_free_runtime(experiment_result);
                t_deadlock_free(iVeh, iSce, iPri) = t_total_deadlock_free;
                n_deadlock_free(iVeh, iSce, iPri) = n_total_deadlock_free;

                avg_speed_step = zeros(n_total_deadlock_free, 1);

                for iStep = 1:n_total_deadlock_free
                    iter = experiment_result.iteration_data(iStep);
                    speeds = [experiment_result.mpa.trims(iter.trim_indices).speed];
                    avg_speed_step(iStep) = mean(speeds);
                end

                avg_speed_scenario(iSce) = mean(avg_speed_step);
            end

            avg_speed_by_veh_pri(iVeh, iPri) = mean(avg_speed_scenario);
        end

    end

    data.x_values = x_values;
    data.t_deadlock_free = t_deadlock_free;
    data.n_deadlock_free = n_deadlock_free;
    data.avg_speed_by_veh_pri = avg_speed_by_veh_pri;
    data.result = res{1, 1, 1};
    data.nVeh = nVeh;
    data.nPri = nPri;
    data.nSce = nSce;
end
