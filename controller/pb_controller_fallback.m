function [u, y_pred, info] = pb_controller_fallback(scenario, u, y_pred, info)
% PB_CONTROLLER    Plan trajectory for one time step using a priority-based controller.
%     Controller simulates multiple distributed controllers.


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
        info.shapes(vehicle_idx,:) = [info.shapes(vehicle_idx,2:end),cell(1,1)];% the last conlumn is used to keep the size
        info.tree_path(vehicle_idx,:) = [info.tree_path(vehicle_idx,2:end),1];% the last element is used to keep the size
        y_pred{vehicle_idx,1} = y_pred{vehicle_idx,1}((scenario.tick_per_step+1)+1:end,:);

        
        
%         disp('info.tree_path');
%         disp(info.tree_path(vehicle_idx,:))
%         disp('info_v.tree_path')
%         disp(info_v.tree_path)
%         disp('next node:')
%         disp(info.next_node)
    end
        
 
end
