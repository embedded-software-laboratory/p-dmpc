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

% This function converts a scenario representation based on the scenario
% and iter structs into a different scenario, in which the selected
% vehicles are represented as obstacles along their predicted path
function [scenario_v, iter_v] = vehicles_as_obstacles(scenario, iter, vehicle_filter, shapes)

    assert( size(shapes,1) == scenario.nVeh-1 )
    assert( length(vehicle_filter) == scenario.nVeh );
    assert( islogical( vehicle_filter ));

    scenario_v = filter_scenario(scenario, ~vehicle_filter);
    iter_v = filter_iter(iter, ~vehicle_filter);
    
    nDynObst = size(scenario_v.dynamic_obstacle_area, 1);
    
    scenario_v.dynamic_obstacle_area = [scenario_v.dynamic_obstacle_area; cell(sum(vehicle_filter),scenario.Hp)];

    for iVeh = 1:sum( vehicle_filter )
        scenario_v.dynamic_obstacle_area(nDynObst+iVeh,:) = shapes(iVeh,:);
    end

end
