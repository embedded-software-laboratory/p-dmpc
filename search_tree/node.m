function n = node(depth, trims, xs, ys, yaws, g_values, h_values)
% NODE  Return new node for search tree and set its values.

    if nargin == 0
        return;
    end
    nVeh = numel(yaws);
    nCols = NodeInfo.n_cols;
    n = zeros(nVeh,nCols);
    n(:,NodeInfo.k) = depth;
    n(:,NodeInfo.trim) = trims;
    n(:,NodeInfo.x) = xs;
    n(:,NodeInfo.y) = ys;
    n(:,NodeInfo.yaw) = yaws;
    n(:,NodeInfo.g) = g_values;
    n(:,NodeInfo.h) = h_values;
end
