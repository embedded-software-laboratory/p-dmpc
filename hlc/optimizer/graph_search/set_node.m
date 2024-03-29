function node = set_node(node, vehicle_filter, sub_node)
    % SET_NODE  Set values to a node stored in a smaller node.

    node(vehicle_filter, NodeInfo.k) = sub_node(:, NodeInfo.k);
    node(vehicle_filter, NodeInfo.trim) = sub_node(:, NodeInfo.trim);
    node(vehicle_filter, NodeInfo.x) = sub_node(:, NodeInfo.x);
    node(vehicle_filter, NodeInfo.y) = sub_node(:, NodeInfo.y);
    node(vehicle_filter, NodeInfo.yaw) = sub_node(:, NodeInfo.yaw);
    node(vehicle_filter, NodeInfo.g) = sub_node(:, NodeInfo.g);
    node(vehicle_filter, NodeInfo.h) = sub_node(:, NodeInfo.h);
end
