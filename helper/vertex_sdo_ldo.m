% finde next vertex in color order with a combination of
% "Saturation Degree Ordering" (SDO) and "Largest Degree Ordering" (LDO)
function idx = vertex_sdo_ldo(A,color,degree)
    max = -1;
    uncolored = find(color == 0);
    for i = uncolored
        d = length(nonzeros(unique(color(A(i,:) == 1))));
        if d > max
            max = d;
            idx = i;
        end
        if d == max
            if degree(i) > degree(idx)
                idx = i;
            end
        end
    end
end

