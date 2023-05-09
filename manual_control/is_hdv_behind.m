function result = is_hdv_behind(lanelet_id_cav, x_cav, lanelet_id_hdv, x_hdv, lanelet_struct)

    result = false;

    adjacent_lanelets_back = [];

    for id = lanelet_id_cav
        adjacent_lanelets_back = unique([adjacent_lanelets_back, lanelet_struct(id).predecessor.refAttribute]);
    end

    hdv_on_predecessor_lanelet = any(ismember(lanelet_id_hdv, adjacent_lanelets_back));

    if hdv_on_predecessor_lanelet
        result = true;
        return;
    end

    hdv_on_same_lanelet = (any(ismember(lanelet_id_hdv, lanelet_id_cav)));

    if hdv_on_same_lanelet
        vector_cav_hdv = x_hdv(1:2) - x_cav(1:2);
        vector_yaw = [cos(x_hdv(3)) sin(x_hdv(3))];
        scalar = dot(vector_yaw, vector_cav_hdv);

        if scalar < 0
            result = true;
            return;
        end

    end

end
