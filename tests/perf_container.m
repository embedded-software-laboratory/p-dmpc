n = 5e2;
nVeh = 5;

%% node array
depth = 0;
trims = ones(nVeh,1);
xs = ones(nVeh,1);
ys = ones(nVeh,1);
yaws = ones(nVeh,1);
g_values = ones(nVeh,1);
h_values = zeros(nVeh,1);
myTree(:,:,n) = node(depth, trims, xs, ys, yaws, g_values, h_values);



%% cell array access performance
% 1 vs 2 indices
na = 50;
A = magic(na);
c1 = cell(numel(A),1);
c2 = cell(size(A));

tic
for col = randperm(na)
    for row = randperm(na)
        el1 = read_element_c1(c1,row,col,na);
    end
end
toc
tic
for col = randperm(na)
    for row = randperm(na)
        el2 = read_element_c2(c2,row,col);
    end
end
toc

tic
for ii = 1:32670
    el = read_element_c2(c2,row,col);
end
toc

        



function el = read_element_c1(c,row,col,na)
el = c{sub2ind([na,na],row,col)};
end
function el = read_element_c2(c,row,col)
el = c{row,col};
end
