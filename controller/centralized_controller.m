function [info,scenario,iter] = centralized_controller(scenario, iter)
% CENTRALIZED_CONTROLLER    Plan trajectory for one time step using a centralized controller.

    % initialize variable to store control results
    info = ControllResultsInfo(scenario.options.amount, scenario.options.Hp, [scenario.vehicles.ID]);
    
    sub_controller = @scenario.sub_controller;
    
    % falsifies controller_runtime slightly
    subcontroller_timer = tic;
    info_v = sub_controller(scenario, iter);
    if info_v.is_exhausted
        % if graph search is exhausted, this vehicles and all vehicles that have directed or
        % undirected couplings with this vehicle will take fallback 
        disp(['Graph search exhausted at time step: ' num2str(iter.k) '.'])
        % all vehicles fall back
        info.vehs_fallback = 1:scenario.options.amount;
        info.is_exhausted(info.vehs_fallback) = true;
    else
        % prepare output data
        info = store_control_info(info, info_v, scenario);
    end

    info.runtime_subcontroller_each_veh = toc(subcontroller_timer);
    % for centralize controller, all vehicles are in the same group
    info.runtime_subcontroller_each_grp = info.runtime_subcontroller_each_veh;
    info.runtime_subcontroller_max = info.runtime_subcontroller_each_veh;
    info.runtime_graph_search_max = info.runtime_subcontroller_each_veh;
end
