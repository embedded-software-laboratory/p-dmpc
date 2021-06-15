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

function visualize_exploration(exploration,scenario)
    info = exploration.info;
    t = [info.tree.node{:}];
    t = reshape(t,scenario.nVeh,NodeInfo.n_cols,[]);
    leaf_idcs = t(1,NodeInfo.k,:)==scenario.Hp;
    leaves = t(:,:,leaf_idcs);
    x = reshape(leaves(:,NodeInfo.x,:),1,[]);
    y = reshape(leaves(:,NodeInfo.y,:),1,[]);
    z = kron(reshape(sum(leaves(:,[NodeInfo.g,NodeInfo.h],:),[1,2]),1,[]),[1,1]);
    line(x,y,z,'Marker','d', 'MarkerSize',6,'LineStyle','none');
end