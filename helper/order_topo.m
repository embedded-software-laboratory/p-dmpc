function order = order_topo(L,C)
% ORDER_TOPO  order topological levels in order to minimize their incoming edges
%     ORDER = ORDER_TOPO(L,C) returns the order of the levels in L regarding the incoming edges in the adjacency C

    lvls = PB_predecessor_groups(L);
    deg = sum(C);
    
    if sum(deg) == 0
        order = [1];
        return;
    end
    
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
