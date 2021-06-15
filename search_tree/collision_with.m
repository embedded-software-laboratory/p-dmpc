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

function collision = collision_with(index, shapes, scenario, iStep)
    collision = false;
    
    obstacles = scenario.obstacles;
            
    nobs = numel(obstacles);
    for i = 1:nobs
        if intersect_sat(shapes{index},obstacles{i}) 
            collision = true;
            return;
        end
    end
    
    if ~isempty(scenario.dynamic_obstacle_area)
        for i = 1:size(scenario.dynamic_obstacle_area,1)
            if intersect_sat(shapes{index},scenario.dynamic_obstacle_area{i,iStep}) 
                collision = true;
                return;
            end
        end
    end
    
    for i = (index - 1) : -1 : 1
        % check if polygons intersect
        if intersect_sat(shapes{i},shapes{index})
            collision = true;
            return;
        end
    end
end
