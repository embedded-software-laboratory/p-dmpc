function one_node = get_cur_node(info,scenario)
    if numel(info) == 1
        assert(numel(info.tree_path)>1);
        one_node = info.tree.Node{info.tree_path(2)};
        return
    end
    trim_indices = [];
    x = [];
    y = [];
    yaw = [];
    for i = 1:scenario.nVeh
        assert(numel(info{i}.tree_path)>1);
        trim_indices = [trim_indices; info{i}.trim_indices];
        x = [x; info{i}.tree.Node{info{i}.tree_path(2)}(1,info{i}.tree.idx.x)];
        y = [y; info{i}.tree.Node{info{i}.tree_path(2)}(1,info{i}.tree.idx.y)];
        yaw = [yaw; info{i}.tree.Node{info{i}.tree_path(2)}(1,info{i}.tree.idx.yaw)];
    end
    init_depth = 0;
    g_values = zeros(scenario.nVeh,1);
    h_values = zeros(scenario.nVeh,1);
    one_node = node( ...
        init_depth, ...
        trim_indices, ...
        x, ...
        y, ...
        yaw, ...
        g_values, ...
        h_values ...
    );
end
