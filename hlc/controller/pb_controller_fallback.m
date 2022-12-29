function info = pb_controller_fallback(info, info_old, scenario, indices_in_vehicle_list)
% pb_controller_fallback    planning by using last priority and trajectories directly

    tick_per_step = scenario.options.tick_per_step + 1;
    
    for vehicle_idx = indices_in_vehicle_list
        if ismember(vehicle_idx, info.vehs_fallback)
            % initialize
            info_v = ControllResultsInfo(1,scenario.options.Hp,scenario.vehicles(vehicle_idx).ID);
            
            info_v.tree = info_old.tree{vehicle_idx};
            info_v.tree_path = del_first_rpt_last(info_old.tree_path(vehicle_idx,:));
            info_v.shapes = del_first_rpt_last(info_old.shapes(vehicle_idx,:));
            info_v.predicted_trims = del_first_rpt_last(info_old.predicted_trims(vehicle_idx,:));
            
            % predicted trajectory of the next time step
            y_pred_v = info_old.y_predicted{vehicle_idx};
            y_pred_v = [y_pred_v(tick_per_step+1:end,:); y_pred_v(tick_per_step*(scenario.options.Hp-1)+1:end,:)];
            info_v.y_predicted = {y_pred_v};
            
            % prepare output data
            info = store_control_info(info,info_v,scenario);
    
            if scenario.options.isPB
                % send message
                predicted_trims = info.predicted_trims(vehicle_idx,:);
                trim_current = predicted_trims(1);
                x0 = info.vehicle_fullres_path{vehicle_idx}(1,indices().x);
                y0 = info.vehicle_fullres_path{vehicle_idx}(1,indices().y);
    
                [predicted_lanelets,~,~] = get_predicted_lanelets(scenario, vehicle_idx, trim_current, x0, y0);
                
                % send message
                scenario.vehicles(vehicle_idx).communicate.traffic.send_message(scenario.k, predicted_trims, predicted_lanelets, info.shapes(vehicle_idx,:), info.vehs_fallback);
            end
        end
    end
        
end
