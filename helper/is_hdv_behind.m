function result = is_hdv_behind(lanelet_id_cav, x_cav, lanelet_id_hdv, x_hdv, lanelets, lanelet_relationships)

    result = false;

    % tolerance is used to decide whether two lanelet point are equal
    % TODO test if tolerance finds all equal lanelet points
    tolerance = 1e-6;

    % initialize array for predecessor lanelets
    predecessor_lanelets = [];

    for id_cav = lanelet_id_cav

        for id_hdv = lanelet_id_hdv

            if id_cav < id_hdv
                relationship = lanelet_relationships{id_cav, id_hdv};
            else
                relationship = lanelet_relationships{id_hdv, id_cav};
            end

            if isempty(relationship)
                continue
            end

            if relationship.type ~= LaneletRelationshipType.longitudinal
                continue
            end

            left_point_start_cav = lanelets{id_cav}(1, [LaneletInfo.lx, LaneletInfo.ly]);
            right_point_start_cav = lanelets{id_cav}(1, [LaneletInfo.rx, LaneletInfo.ry]);
            center_point_start_cav = lanelets{id_cav}(1, [LaneletInfo.cx, LaneletInfo.cy]);
            left_point_end_hdv = lanelets{id_hdv}(end, [LaneletInfo.lx, LaneletInfo.ly]);
            right_point_end_hdv = lanelets{id_hdv}(end, [LaneletInfo.rx, LaneletInfo.ry]);
            center_point_end_hdv = lanelets{id_hdv}(end, [LaneletInfo.cx, LaneletInfo.cy]);

            if ( ...
                    ... % lanelets are not longitudinal consecutive
                    norm(center_point_start_cav - center_point_end_hdv) > tolerance && ...
                    ... % lanelets are not diagonal consecutive
                    norm(left_point_start_cav - right_point_end_hdv) > tolerance && ...
                    norm(right_point_start_cav - left_point_end_hdv) > tolerance ...
                )
                continue
            end

            predecessor_lanelets = [predecessor_lanelets, id_hdv];

        end

    end

    % delete doubled lanelets
    predecessor_lanelets = unique(predecessor_lanelets);

    % each lanelet should have at least one predecessor
    assert(~isempty(predecessor_lanelets))

    hdv_on_predecessor_lanelet = any(ismember(lanelet_id_hdv, predecessor_lanelets));

    if hdv_on_predecessor_lanelet
        result = true;
        return;
    end

    hdv_on_same_lanelet = any(ismember(lanelet_id_hdv, lanelet_id_cav));

    if hdv_on_same_lanelet
        vector_cav_hdv = x_hdv(1:2) - x_cav(1:2);
        vector_yaw = [cos(x_hdv(3)) sin(x_hdv(3))];
        scalar = dot(vector_yaw, vector_cav_hdv);

        if scalar < 0
            % hdv is on same lanelet behind cav
            result = true;
            return;
        end

    end

end
