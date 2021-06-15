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

function scenario = recursive_feasibility_scenario(recursive_feasibility,is_ok)
    scenario = Scenario();
    veh = Vehicle();
    radius = 2;
    center_x = 2.25;
    center_y = 2;
    veh.x_start = -radius + center_x;
    veh.y_start = 0 + center_y;
    veh.yaw_start = 0;
    veh.x_goal = radius + center_x;
    veh.y_goal = 0 + center_y;
    veh.yaw_goal = 0;
    veh.trim_config = 1;
    veh.referenceTrajectory = [veh.x_start veh.y_start;veh.x_goal veh.y_goal];
    scenario.vehicles = veh;
    scenario.nVeh = 1;
    scenario.Hp = 2;
    scenario.T_end = 6;
    scenario.trim_set = 1;
    
    scenario.plot_limits = [-0.5,5;1.5,2.5];

    scenario.model = BicycleModel(veh.Lf,veh.Lr);

    scenario.mpa = MotionPrimitiveAutomaton(...
        scenario.model...
        , scenario.trim_set...
        , scenario.offset...
        , scenario.dt...
        , scenario.nVeh...
        , scenario.Hp...
        , scenario.tick_per_step...
        , recursive_feasibility...
    );

    scenario.name = sprintf('recursive_feasibility_%s', mat2str(recursive_feasibility));


    x_obs_l = veh.x_start+0.5*veh.Length...
        + ( (scenario.mpa.trims(1).speed + scenario.mpa.trims(2).speed) / 2 ...
            +(scenario.mpa.trims(2).speed + scenario.mpa.trims(3).speed) / 2 ...
            + 1*scenario.mpa.trims(3).speed ...
            +(scenario.mpa.trims(2).speed + scenario.mpa.trims(3).speed) / 2 ...
        ) * scenario.dt ...
        + scenario.offset + 0.08;


    scenario.obstacles{1} = [
        0    0.2  0.2  0
        -0.2 -0.2  0.2  0.2] ...
        + [x_obs_l; center_y];
    if is_ok
        scenario.obstacles{1} = scenario.obstacles{1} + [0.3; 0];
    end

end