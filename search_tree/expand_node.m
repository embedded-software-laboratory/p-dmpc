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

function [expanded_nodes] = expand_node(scenario, iter, cur_node)
    trim_tuple = scenario.mpa.trim_tuple;
    trim_length = length(scenario.mpa.trims)*ones(1, scenario.nVeh);
    cur_trim_id = tuple2index(cur_node(:,NodeInfo.trim),trim_length);
    k_cur = cur_node(1,NodeInfo.k);
    k_exp = k_cur + 1;
    successor_trim_ids = find(scenario.mpa.transition_matrix(cur_trim_id, :, k_exp));
    
    nTrims = numel(successor_trim_ids);
    expanded_nodes = cell(1, nTrims);
    % Preallocate maneuver
    for iTrim = 1:nTrims
        id = successor_trim_ids(iTrim);
        expanded_node = cur_node;
        expanded_node(:,NodeInfo.k) = k_exp;
        expanded_node(:,NodeInfo.trim) = trim_tuple(id,:);
        for iVeh = 1 : scenario.nVeh
            itrim1 = cur_node(iVeh,NodeInfo.trim);
            itrim2 = expanded_node(iVeh,NodeInfo.trim);
            maneuver = scenario.mpa.maneuvers{itrim1, itrim2};
            c = cos(cur_node(iVeh,NodeInfo.yaw));
            s = sin(cur_node(iVeh,NodeInfo.yaw));

            expanded_node(iVeh,NodeInfo.x) = c*maneuver.dx - s*maneuver.dy + cur_node(iVeh,NodeInfo.x);
            expanded_node(iVeh,NodeInfo.y) = s*maneuver.dx + c*maneuver.dy + cur_node(iVeh,NodeInfo.y);
            expanded_node(iVeh,NodeInfo.yaw) = cur_node(iVeh,NodeInfo.yaw) + maneuver.dyaw;
        end

        % expanded_node = calculate_next_values_reference(scenario, iter, expanded_node, NodeInfo);
        % Cost to come
        % iter.reference is of size (scenario.nVeh,scenario.Hp,2)
        % Distance to reference trajectory points squared
        for iVeh = 1:scenario.nVeh
            expanded_node(iVeh,NodeInfo.g) = expanded_node(iVeh,NodeInfo.g) ...
                + norm(...
                    [expanded_node(iVeh,NodeInfo.x)...
                        - iter.referenceTrajectoryPoints(iVeh,k_exp,1);...
                     expanded_node(iVeh,NodeInfo.y)...
                        - iter.referenceTrajectoryPoints(iVeh,k_exp,2)]...
                )^2;
        end
        
        % Cost to go
        expanded_node(:,NodeInfo.h) = 0;
        % Distance to every reference trajectory point squared
        % subtract squared distance traveled for every timestep and vehicle
        time_steps_to_go = scenario.Hp - k_exp;
        for iVeh = 1:scenario.nVeh
            d_traveled_max = 0;
            for i_t = 1:time_steps_to_go
                d_traveled_max = d_traveled_max...
                    + scenario.dt*iter.vRef(iVeh,k_exp+i_t);
                expanded_node(iVeh,NodeInfo.h) = expanded_node(iVeh,NodeInfo.h)...
                    + (norm( [expanded_node(iVeh,NodeInfo.x)-iter.referenceTrajectoryPoints(iVeh,k_exp+i_t,1);...
                              expanded_node(iVeh,NodeInfo.y)-iter.referenceTrajectoryPoints(iVeh,k_exp+i_t,2)] ) ...
                        - d_traveled_max)^2;
            end
        end

        expanded_nodes{iTrim} = expanded_node;
    end
end