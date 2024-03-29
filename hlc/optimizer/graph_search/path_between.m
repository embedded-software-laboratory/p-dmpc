function search_path = path_between(iCur, iNext, tree, scenario)
    %PATH_BETWEEN Return path as a cell array between two nodes

    n_veh = size(tree.x, 1);
    search_path = cell(1, n_veh);

    for iVeh = 1:n_veh
        maneuver = scenario.mpa.maneuvers{tree.trim(iVeh, iCur), tree.trim(iVeh, iNext)};

        assert(~isempty(maneuver), 'manuevers{%d, %d} is empty.', tree.trim(iVeh, iCur), tree.trim(iVeh, iNext));

        x = tree.x(iVeh, iCur);
        y = tree.y(iVeh, iCur);
        yaw = tree.yaw(iVeh, iCur);

        xs = maneuver.xs;
        ys = maneuver.ys;
        yaws = maneuver.yaws + yaw;

        length_maneuver = length(xs);
        trims = tree.trim(iVeh, iCur) * ones(length_maneuver, 1);

        [xs, ys] = translate_global(yaw, x, y, xs, ys);

        search_path(iVeh) = {[xs', ys', yaws', trims]};
    end

end
