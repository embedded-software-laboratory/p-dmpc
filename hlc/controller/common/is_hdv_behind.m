function result = is_hdv_behind(lanelet_id_cav, cav_measurement, lanelet_id_hdv, hdv_measurement, lanelets, lanelet_relationships)

    result = false;

    % tolerance is used to decide whether two lanelet point are equal
    tolerance = 1e-6;

    % initialize array for predecessor lanelets
    predecessor_lanelets = [];
    % initialize array for merging lanelets
    overlapping_lanelets = [];

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

            if ( ...
                    relationship.type == LaneletRelationshipType.merging || ...
                    relationship.type == LaneletRelationshipType.forking ...
                )
                overlapping_lanelets = [overlapping_lanelets, id_hdv];
                continue
            elseif relationship.type ~= LaneletRelationshipType.longitudinal
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

    % delete doubled lanelets
    overlapping_lanelets = unique(overlapping_lanelets);

    hdv_on_same_lanelet = any(ismember(lanelet_id_hdv, lanelet_id_cav));
    hdv_on_overlapping_lanelet = ~isempty(overlapping_lanelets);

    if hdv_on_same_lanelet || hdv_on_overlapping_lanelet
        vector_cav_hdv = [hdv_measurement.x, hdv_measurement.y] - [cav_measurement.x, cav_measurement.y];
        vector_yaw = [cos(hdv_measurement.yaw) sin(hdv_measurement.yaw)];
        scalar = dot(vector_yaw, vector_cav_hdv);

        if scalar < 0
            % hdv is on same/overlapping lanelet behind cav
            result = true;
            return;
        end

    end

end
