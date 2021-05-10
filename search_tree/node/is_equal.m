function cond = is_equal(next_node, comp_node)
    cond = (all(next_node.xs == comp_node.xs) && all(next_node.ys == comp_node.ys) && all(next_node.yaws == comp_node.yaws));
end

