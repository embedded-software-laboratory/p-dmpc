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

function scenario = circle_scenario_pb(nVeh)
    scenario = Scenario();
    
    radius = 2;
    yaws = pi*2/nVeh*(0:nVeh-1);
    for yaw = yaws
        s = sin(yaw);
        c = cos(yaw);
        veh = Vehicle();

        veh.trim_config = 1;

        veh.x_start = -c*radius;
        veh.y_start = -s*radius;
        veh.yaw_start = yaw;
        veh.x_goal = c*radius;
        veh.y_goal = s*radius;
        veh.yaw_goal = yaw;
        % Lab: translate by center
        center_x = 2.25;
        center_y = 2;
        veh.x_start = veh.x_start + center_x;
        veh.y_start = veh.y_start + center_y;
        veh.x_goal  = veh.x_goal  + center_x;
        veh.y_goal  = veh.y_goal  + center_y;

        veh.referenceTrajectory = [veh.x_start veh.y_start
                                   veh.x_goal  veh.y_goal];

        scenario.vehicles = [scenario.vehicles, veh];
    end
    if nVeh <= 2
        scenario.plot_limits = [-0.5,5;1.5,2.5];
    else
        scenario.plot_limits = [-0.5,5;-0.5,4.5];
    end
    scenario.nVeh = nVeh;
    scenario.name = sprintf('%i-circle', scenario.nVeh);

    scenario.model = BicycleModel(veh.Lf,veh.Lr);

    scenario.T_end = 6;

    recursive_feasibility = true;
    scenario.mpa = MotionPrimitiveAutomaton(...
        scenario.model...
        , scenario.trim_set...
        , scenario.offset...
        , scenario.dt...
        , 1 ...
        , scenario.Hp...
        , scenario.tick_per_step...
        , recursive_feasibility...
    );

       if scenario.assignPrios
            scenario.coupling_adjacency = triu(ones(nVeh))-eye(nVeh);
       else
            scenario.coupling_adjacency = triu(ones(nVeh))-eye(nVeh);
       end
       scenario.controller_name = strcat(scenario.controller_name, '-PB');
       scenario.controller = @(s,i) pb_controller(s,i);

end