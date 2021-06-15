% MIT License
% 
% Copyright (c) 2021 Lehrstuhl Informatik 11 - RWTH Aachen University
% 
% Permission is hereby granted, free of charge, to any person obtaining a copy
% of this software and associated documentation files (the "Software"), to deal
% in the Software without restriction, including without limitation the rights
% to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
% copies of the Software, and to permit persons to whom the Software is
% furnished to do so, subject to the following conditions:
% 
% The above copyright notice and this permission notice shall be included in all
% copies or substantial portions of the Software.
% 
% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
% IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
% FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
% AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
% LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
% OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
% SOFTWARE.
% 
% This file is part of receding-horizon-graph-search.
% 
% Author: i11 - Embedded Software, RWTH Aachen University

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