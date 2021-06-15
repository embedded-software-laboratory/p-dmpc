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

% Author: Janis Maczijewski
% Date: Sep 2015
%% Creates a rectangle with the given position and orientation.
function [ polygon_XY ] = transformedRectangle( x,y,angle, Length, Width )

    unitSquare = [ 0 0 0 1; 1 0 0 1; 1 1 0 1; 0 1 0 1]';
    
    % Read this bottom-up
    polygon_XY =  makehgtform('translate',[x y 0]) ...
                * makehgtform('zrotate',angle) ...
                * makehgtform('scale',[Length Width 1]) ...
                * makehgtform('translate',[-.5 -.5 0]) ...
                * unitSquare;
            
    polygon_XY = polygon_XY(1:2,:);
end

