function info = pb_controller_fallback(info, info_old, scenario)
% pb_controller_fallback    planning by using last priority and trajectories directly

    vehs_fallback = info.vehs_fallback;
    vehs_fallback = vehs_fallback(:)'; % turn to row vector
    assert(isrow(vehs_fallback))

    tick_per_step = scenario.tick_per_step + 1;
    
    for vehicle_idx = vehs_fallback    
        % initialize
        info_v = ControllResultsInfo(1,scenario.Hp,scenario.vehicles(vehicle_idx).ID);
        
        info_v.tree = info_old.tree{vehicle_idx};
        info_v.tree_path = del_first_rpt_last(info_old.tree_path(vehicle_idx,:));
        info_v.shapes = del_first_rpt_last(info_old.shapes(vehicle_idx,:));
        info_v.predicted_trims = del_first_rpt_last(info_old.predicted_trims(vehicle_idx,:));
        
        % predicted trajectory of the next time step
        y_pred_v = info_old.y_predicted{vehicle_idx};
        y_pred_v = [y_pred_v(tick_per_step+1:end,:); y_pred_v(tick_per_step*(scenario.Hp-1)+1:end,:)];
        info_v.y_predicted = {y_pred_v};
        
        % prepare output data
        info = store_control_info(info,info_v,scenario);

        if scenario.options.isParl
            % send message
            predicted_trims = info.predicted_trims(vehicle_idx,:);
            trim_current = predicted_trims(1);
            x0 = info.vehicle_fullres_path{vehicle_idx}(1,indices().x);
            y0 = info.vehicle_fullres_path{vehicle_idx}(1,indices().y);
            [predicted_lanelets,~,~] = get_predicted_lanelets(scenario.vehicles(vehicle_idx), trim_current, x0, y0, scenario.mpa, scenario.dt, scenario.options.isParl, scenario.name);
            % send message
            send_message(scenario.vehicles(vehicle_idx).communicate, scenario.k, predicted_trims, predicted_lanelets, info.shapes(vehicle_idx,:));
        end

    end
        
end
