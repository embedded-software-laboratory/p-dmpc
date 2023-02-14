function collision = intersect_lanelets(shape, lanelet)
% INTERSECT_LANLETS     Wrapper for INTERSECT_SAT calls for a lanlet segment, devided into left and right bound.

    collision = false;
    nSeg = size(lanelet,1)-1;
    for i = 1:nSeg
        % right bound
        if intersect_sat(shape,lanelet(i:i+1,[LaneletInfo.rx,LaneletInfo.ry])')
            collision = true;
            return;
        end
        % left bound
        if intersect_sat(shape,lanelet(i:i+1,[LaneletInfo.lx,LaneletInfo.ly])')
            collision = true;
            return;
        end
    end
end

