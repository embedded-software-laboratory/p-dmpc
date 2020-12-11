function dist = euclidean_distance(cur_node, target_poses)
%EUCLIDEAN_DISTANCE Computes euclidean distance of two given poses
dist = sqrt((cur_node.xs - target_poses.xs).^2 + (cur_node.ys - target_poses.ys).^2);
end

