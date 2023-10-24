function search_path = path_between(iCur, iNext, tree, mpa)
    %PATH_BETWEEN Return path as a cell array between two nodes

    n_veh = tree.number_of_vehicles();
    search_path = cell(1, n_veh);

    for iVeh = 1:n_veh
        maneuver = mpa.maneuvers{tree.get_trim(iVeh, iCur), tree.get_trim(iVeh, iNext)};

        x = tree.get_x(iVeh, iCur);
        y = tree.get_y(iVeh, iCur);
        yaw = tree.get_yaw(iVeh, iCur);

        xs = maneuver.xs;
        ys = maneuver.ys;
        yaws = maneuver.yaws + yaw;

        length_maneuver = length(xs);
        trims = double(tree.get_trim(iVeh, iCur)) * ones(length_maneuver, 1);

        [xs, ys] = translate_global(yaw, x, y, xs, ys);

        search_path(iVeh) = {[xs', ys', yaws', trims]};
    end

end
