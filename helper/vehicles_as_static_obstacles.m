function scenario_v = vehicles_as_static_obstacles(scenario_v, iter, all_coupled_vehs_LP)
% VEHICLES_AS_STATIC_OBSTACLES Add adjacent vehicles with lower priorities
% as static obstacles.

    for i = 1:length(all_coupled_vehs_LP)
        veh_index = all_coupled_vehs_LP(i);
        x0 = iter.x0(veh_index, indices().x);
        y0 = iter.x0(veh_index, indices().y);
        yaw0 = iter.x0(veh_index, indices().heading);
        veh = Vehicle();
        x_locals = [-1, -1,  1,  1] * (veh.Length/2+scenario_v.offset);
        y_locals = [-1,  1,  1, -1] * (veh.Width/2+scenario_v.offset);
        [x_globals,y_globals] = translate_global(yaw0, x0, y0, x_locals, y_locals);
        scenario_v.obstacles(end+1) = {[x_globals;y_globals]};
    end

end