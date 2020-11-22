function cond = cheaper(node1,node2)
    cond = false;
    value1 = sum(node1.g_values + node1.h_values);
    value2 = sum(node2.g_values + node2.h_values); 

    if value1 < value2
        cond = true;
    end
end

