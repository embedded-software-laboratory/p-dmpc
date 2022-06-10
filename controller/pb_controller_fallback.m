function [u, y_pred, info, scenario] = pb_controller_fallback(scenario, iter, u, y_pred, info, options)
% pb_controller_fallback    planning by using last priority and trajectories directly



    nVeh = length(scenario.vehicles);
    for vehicle_idx = 1:nVeh
        subcontroller_timer = tic;
        %
        info_v.tree = info.tree{vehicle_idx};
        info_v.tree_path = info.tree_path(vehicle_idx,2:end);
        info_v.shapes = info.shapes;
        info_v.trim_indices = info.trim_indices(vehicle_idx,:);
        


        % prepare output data
        info.subcontroller_runtime(vehicle_idx) = toc(subcontroller_timer);
        info.n_expanded = info.n_expanded;
        info.next_node = set_node(info.next_node,vehicle_idx,info_v);
        info.vehicle_fullres_path(vehicle_idx) = path_between(info_v.tree_path(1),info_v.tree_path(2),info_v.tree,scenario.mpa);
        info.trim_indices(vehicle_idx,:) = info_v.trim_indices;
        info.shapes(vehicle_idx,:) = [info.shapes(vehicle_idx,2:end),info.shapes(vehicle_idx,end)]; % the last conlumn is used to keep the size
        info.tree_path(vehicle_idx,:) = [info.tree_path(vehicle_idx,2:end),info.tree_path(vehicle_idx,end)]; % the last element is used to keep the size
        info.predicted_trims(vehicle_idx,:) = [info.predicted_trims(vehicle_idx,2:end),info.predicted_trims(vehicle_idx,end)]; % the last element is used to keep the size
        y_pred{vehicle_idx,1} = [y_pred{vehicle_idx,1}((scenario.tick_per_step+1)+1:end,:);y_pred{vehicle_idx,1}((scenario.tick_per_step+1)*(scenario.Hp-1)+1:end,:)];

   
%         disp('info.tree_path');
%         disp(info.tree_path(vehicle_idx,:))
%         disp('info_v.tree_path')
%         disp(info_v.tree_path)
%         disp('next node:')
%         disp(info.next_node)

    end

    if options.isParl
        % send message

        info.subcontroller_runtime_all_grps = 0;
        for iVeh = 1:nVeh
            predicted_trims = info.predicted_trims(iVeh,:);
            trim_current = predicted_trims(1);
            x0 = info.vehicle_fullres_path{iVeh}(1,indices().x);
            y0 = info.vehicle_fullres_path{iVeh}(1,indices().y);
            [predicted_lanelets,~,~] = get_predicted_lanelets(scenario.vehicles(iVeh), trim_current, x0, y0, scenario.mpa, scenario.dt);
            % send message
            send_message(scenario.vehicles(iVeh).communicate, scenario.k, predicted_trims, predicted_lanelets, info.shapes(iVeh,:));
        end
    end
        
end
