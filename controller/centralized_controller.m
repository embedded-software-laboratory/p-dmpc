function [hlc] = centralized_controller(hlc)
% CENTRALIZED_CONTROLLER    Plan trajectory for one time step using a centralized controller.

    % initialize variable to store control results
    hlc.info = ControllResultsInfo(hlc.scenario.options.amount, hlc.scenario.options.Hp, [hlc.scenario.vehicles.ID]);
    
    sub_controller = @hlc.sub_controller;
    
    % falsifies controller_runtime slightly
    subcontroller_timer = tic;
    info_v = sub_controller(hlc.scenario, hlc.iter);
    if info_v.is_exhausted
        % if graph search is exhausted, this vehicles and all vehicles that have directed or
        % undirected couplings with this vehicle will take fallback 
        disp(['Graph search exhausted at time step: ' num2str(hlc.scenario.k) '.'])
        % all vehicles fall back
        hlc.info.vehs_fallback = 1:hlc.scenario.options.amount;
        hlc.info.is_exhausted(hlc.info.vehs_fallback) = true;
    else
        % prepare output data
        hlc.info = store_control_info(hlc.info, info_v, hlc.scenario);
    end

    hlc.info.runtime_subcontroller_each_veh = toc(subcontroller_timer);
    % for centralize controller, all vehicles are in the same group
    hlc.info.runtime_subcontroller_each_grp = hlc.info.runtime_subcontroller_each_veh;
    hlc.info.runtime_subcontroller_max = hlc.info.runtime_subcontroller_each_veh;
    hlc.info.runtime_graph_search_max = hlc.info.runtime_subcontroller_each_veh;
end
