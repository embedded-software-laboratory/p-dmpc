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

function expanded_node = calculate_next_values_reference(scenario, iter, expanded_node)
    % Cost to come
    % iter.reference is of size (scenario.nVeh,scenario.Hp,2)
    % Distance to reference trajectory points squared
    for iVeh = 1:scenario.nVeh
        expanded_node(iVeh,NodeInfo.g) = expanded_node(iVeh,NodeInfo.g) ...
            + norm( [expanded_node(iVeh,NodeInfo.x) - iter.referenceTrajectoryPoints(iVeh,expanded_node(1,NodeInfo.k),1);...
                     expanded_node(iVeh,NodeInfo.y) - iter.referenceTrajectoryPoints(iVeh,expanded_node(1,NodeInfo.k),2)] );                     
    end
    
    % Cost to go
    expanded_node(:,NodeInfo.h) = 0;
    if (expanded_node(1,NodeInfo.k) < scenario.Hp)
        % Distance to every reference trajectory point squared
        % subtract squared distance traveled for every timestep and vehicle
        time_steps_to_go = scenario.Hp - expanded_node(1,NodeInfo.k);
        for iVeh = 1:scenario.nVeh
            d_traveled_per_step = scenario.dt*iter.vRef(iVeh);
            for i_t = 1:time_steps_to_go
                expanded_node(iVeh,NodeInfo.h) = expanded_node(iVeh,NodeInfo.h)...
                    + norm( [expanded_node(iVeh,NodeInfo.x)-iter.referenceTrajectoryPoints(iVeh,expanded_node(1,NodeInfo.k)+i_t,1);...
                             expanded_node(iVeh,NodeInfo.y)-iter.referenceTrajectoryPoints(iVeh,expanded_node(1,NodeInfo.k)+i_t,2)] ) ...
                    - d_traveled_per_step * i_t;
            end
        end
    end
end