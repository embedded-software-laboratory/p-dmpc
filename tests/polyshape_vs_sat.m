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