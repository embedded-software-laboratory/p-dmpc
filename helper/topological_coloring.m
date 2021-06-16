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

% find a topological sorting with a minimum number of levels (approximation)
% based on graph coloring in an undirected graph
function [valid,L] = topological_coloring(A)
    % init
    a = length(A(:,1));
    col = 1:a;
    color = zeros(1,a);
    cum_matrix = cumsum(A);
    degree = cum_matrix(end,:);
    % assignment
    color(degree == 0) = 1;
    while ~all(color ~= 0)
        % get next vertex in coloring order
        v = vertex_sdo_ldo(A,color,degree);
        neighbor_col  = unique(color(A(v,:) == 1));
        poss_col = setdiff(col,neighbor_col);
        color(v) = poss_col(1);
    end
    % topolical sorting matrix
    used_col = unique(color);
    k_col = length(used_col);
    L = zeros(k_col,a);
    for i = 1 : k_col
        L(i,color == used_col(i)) = 1;
    end
    valid = isequal(sum(sum(L,1)),a);
end

