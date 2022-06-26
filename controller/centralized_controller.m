function [info,scenario] = centralized_controller(scenario, iter)
% CENTRALIZED_CONTROLLER    Plan trajectory for one time step using a centralized controller.

    % initialize variable to store control results
    info = ControllResultsInfo(scenario.nVeh, scenario.Hp, [scenario.vehicles.ID]);
    
    sub_controller = @(scenario, iter)...
        graph_search(scenario, iter);
    
    % falsifies controller_runtime slightly
    subcontroller_timer = tic;
    info_v = sub_controller(scenario, iter);
    if info_v.is_exhausted
        % if graph search is exhausted, this vehicles and all vehicles that have directed or
        % undirected couplings with this vehicle will take fallback 
        disp(['Graph search exhausted at time step: ' num2str(scenario.k) '.'])
        % all vehicles fall back
        info.vehs_fallback = 1:scenario.nVeh;
        info.is_exhausted(info.vehs_fallback) = true;
    else
        % prepare output data
        info = store_control_info(info, info_v, scenario);
    end

    info.subcontroller_runtime = toc(subcontroller_timer);
    % for centralize controller, all vehicles are in the same group
    info.subcontroller_runtime_all_grps = info.subcontroller_runtime;
end
