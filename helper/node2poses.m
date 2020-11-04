function poses = node2poses(node)
    poses=repmat(struct('x',0),length(node.xs),1);
    tmp = num2cell(node.xs);
    [poses.x] = tmp{:};
    tmp = num2cell(node.ys);
    [poses.y] = tmp{:};
    tmp = num2cell(node.yaws);
    [poses.yaw] = tmp{:};
end
