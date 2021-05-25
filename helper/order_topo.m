% input: topological level matrix L and coupling matrix C
% output: order for topological levels regarding incoming edges
function order = order_topo(L,C)
    lvls = PB_predecessor_groups(L);
    deg = sum(C);
    order = [];
    while sum(deg) ~= 0
        max_deg = 0;
        max_idx = 0;
        for i = 1 : length(deg)
            if deg(i) > max_deg
                max_deg = deg(i);
                max_idx = i;
            end
        end
        lvl = 0;
        for j = 1 : length(lvls)
            if ismember(max_idx,lvls(j).members)
                lvl = j;
                break;
            end
        end
        order = [order, lvl];
        deg(lvls(lvl).members) = 0;
    end 
end
