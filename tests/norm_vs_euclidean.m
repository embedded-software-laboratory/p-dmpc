n = 1e7;

sx = 1;
sy = 1;
tx = 2;
ty = 2;

sn = [1;1];
tn = [2;2];

for i=1:n
    a = euclidean_distance(sx,sy,tx,ty);
end
for i=1:n
    an = norm(sn-tn);
end

a
an