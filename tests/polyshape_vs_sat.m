pgon = polyshape([0 0 1 1],[1 0 0 1]);
pgon_overlap = polyshape([0 0 1 1]+0.5,[1 0 0 1]);
pgon_no_overlap = polyshape([0 0 1 1]+2,[1 0 0 1]);
vpgon = [[0 0 1 1];[1 0 0 1]];
vpgon_overlap = [[0 0 1 1]+0.5;[1 0 0 1]];
vpgon_no_overlap = [[0 0 1 1]+2;[1 0 0 1]];
N = 1e4;

for i = 1:N
    is_collide = intersect_sat(vpgon,vpgon_overlap);
end
for i = 1:N
    is_intersect = intersect(pgon,pgon_overlap);
    if is_intersect.NumRegions ~= 0   
            is_intersect = true;
    end
end
for i = 1:N
    is_not_collide = intersect_sat(vpgon,vpgon_no_overlap);
end
for i = 1:N
    is_not_intersect = intersect(pgon,pgon_no_overlap);
    if is_not_intersect.NumRegions == 0   
            is_not_intersect = false;
    end
end
