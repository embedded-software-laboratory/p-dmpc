function dist = euclidean_distance(x1,y1,x2,y2)
%EUCLIDEAN_DISTANCE Computes euclidean distance in 2D elementwise, result is vector
dist = sqrt( (x1-x2).^2 + (y1-y2).^2);
end

