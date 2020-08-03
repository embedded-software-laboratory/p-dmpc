function dist = euclidean_distance(ref, target)
%EUCLIDEAN_DISTANCE Computes euclidean distance of two given poses
dist = sqrt((ref.x - target.x)^2 + (ref.y - target.y)^2);
end

