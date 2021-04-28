function n = node(depth, trims, xs, ys, yaws, g_values, h_values)
    if nargin == 0
        return;
    end
    nVeh = numel(yaws);
    idx = tree.nodeCols();
    nCols = numel(fieldnames(idx));
    n = zeros(nVeh,nCols);
    n(:,idx.depth) = depth;
    n(:,idx.trim) = trims;
    n(:,idx.x) = xs;
    n(:,idx.y) = ys;
    n(:,idx.yaw) = yaws;
    n(:,idx.g) = g_values;
    n(:,idx.h) = h_values;
end