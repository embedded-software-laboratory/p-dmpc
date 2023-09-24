function pb_controller_fallback(obj)
    % pb_controller_fallback    planning by using last priority and trajectories directly

    % input argument: obj - handle of high level controller
    % TODO should be handle of prioritized controller (see issue #142)

    tick_per_step = obj.scenario.options.tick_per_step + 1;

    for vehicle_idx = obj.plant.indices_in_vehicle_list

        if ismember(vehicle_idx, obj.info.vehs_fallback)
            % initialize
            info_v = ControlResultsInfo(1, obj.scenario.options.Hp, obj.plant.all_vehicle_ids(vehicle_idx));

            info_v.tree = obj.info_old.tree{vehicle_idx};
            info_v.tree_path = del_first_rpt_last(obj.info_old.tree_path(vehicle_idx, :));
            info_v.shapes = del_first_rpt_last(obj.info_old.shapes(vehicle_idx, :));
            info_v.predicted_trims = del_first_rpt_last(obj.info_old.predicted_trims(vehicle_idx, :));

            % predicted trajectory of the next time step
            y_pred_v = obj.info_old.y_predicted{vehicle_idx};
            y_pred_v = [y_pred_v(tick_per_step + 1:end, :); y_pred_v(tick_per_step * (obj.scenario.options.Hp - 1) + 1:end, :)];
            info_v.y_predicted = {y_pred_v};

            % prepare output data
            obj.info = store_control_info(obj.info, info_v, obj.scenario, obj.mpa);

            % data only need to be updated if isDealPredictionInconsistency
            % is off, because only old reachable sets but no old predicted areas
            % are used by controller
            if obj.scenario.options.is_prioritized && obj.scenario.options.isDealPredictionInconsistency == false
                % send message
                obj.predictions_communication{vehicle_idx}.send_message(obj.iter.k, obj.info.shapes(vehicle_idx, :), obj.info.vehs_fallback);
            end

        end

    end

end
