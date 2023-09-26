function info = pb_controller_fallback(iter, info, info_old, scenario, mpa, all_vehicle_ids, indices_in_vehicle_list)
    % pb_controller_fallback    planning by using last priority and trajectories directly

    tick_per_step = scenario.options.tick_per_step + 1;

    for vehicle_idx = indices_in_vehicle_list

        if ismember(vehicle_idx, info.vehs_fallback)
            % initialize
            info_v = ControlResultsInfo(1, scenario.options.Hp, all_vehicle_ids(vehicle_idx));

            info_v.tree = info_old.tree{vehicle_idx};
            info_v.tree_path = del_first_rpt_last(info_old.tree_path(vehicle_idx, :));
            info_v.shapes = del_first_rpt_last(info_old.shapes(vehicle_idx, :));
            info_v.predicted_trims = del_first_rpt_last(info_old.predicted_trims(vehicle_idx, :));

            % predicted trajectory of the next time step
            y_pred_v = info_old.y_predicted{vehicle_idx};
            y_pred_v = [y_pred_v(tick_per_step + 1:end, :); y_pred_v(tick_per_step * (scenario.options.Hp - 1) + 1:end, :)];
            info_v.y_predicted = {y_pred_v};

            % prepare output data
            info = store_control_info(info, info_v, scenario, mpa);

            % data only need to be updated if isDealPredictionInconsistency
            % is off, because only old reachable sets but no old predicted areas
            % are used by controller
            if scenario.options.is_prioritized && scenario.options.isDealPredictionInconsistency == false
                % send message
                scenario.vehicles(vehicle_idx).communicate.predictions.send_message(iter.k, info.shapes(vehicle_idx, :), info.vehs_fallback);
            end

        end

    end

end
