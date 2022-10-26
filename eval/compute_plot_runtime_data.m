function [ data ] = compute_plot_runtime_data(res)
% GET_PLOT_RUNTIME_DATA  Calculate data needed to export plots for runtime evaluation

    data = struct();

    [nVeh, nPri, nSce ] = size(res);
    
    x_values = zeros(1, nVeh);
    t_deadlock_free = zeros(nVeh, nSce, nPri);
    n_deadlock_free = zeros(nVeh, nSce, nPri);
    for iVeh = 1:nVeh
        x_values(iVeh) = res{iVeh,1,1}.scenario.options.amount;
        for iPri = 1:nPri
            for iSce = 1:nSce
                disp(['Scenario ',num2str(iVeh),' / ',num2str(iPri),' / ',num2str(iSce)])
                result = res{iVeh,iPri,iSce};
                [n_total_deadlock_free,t_total_deadlock_free] = compute_deadlock_free_runtime(result);
                t_deadlock_free(iVeh, iSce, iPri) = t_total_deadlock_free;
                n_deadlock_free(iVeh, iSce, iPri) = n_total_deadlock_free;
            end
        end
    end

    data.x_values = x_values;
    data.t_deadlock_free = t_deadlock_free;
    data.n_deadlock_free = n_deadlock_free;
    data.result = res{1,1,1};
    data.nVeh = nVeh;
    data.nPri = nPri;
    data.nSce = nSce;
    
end

