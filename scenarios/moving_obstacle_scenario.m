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

function scenario = moving_obstacle_scenario()
    scenario = Scenario();
    scenario.trim_set = 3;
    veh = Vehicle();
    radius = 3;
    center_x = 2.25;
    center_y = 0;
    veh.x_start = -radius + center_x - 2;
    veh.y_start = 0 + center_y;
    veh.yaw_start = 0;
    veh.x_goal = radius + center_x + 100;
    veh.y_goal = 0 + center_y;
    veh.yaw_goal = 0;
    veh.trim_config = 1;
    veh.referenceTrajectory = [veh.x_start veh.y_start;veh.x_goal veh.y_goal];
    scenario.vehicles = veh;
    scenario.nVeh = 1;
    scenario.Hp = 5;
    scenario.T_end = 14;
    
    scenario.model = BicycleModel(veh.Lf,veh.Lr);
    
    scenario.plot_limits = [-3.5,6.5;-1.5,1.5];
    
    recursive_feasibility = true;
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

    x1_obs = veh.x_start + 4;
    x2_obs = x1_obs + 1.5;
    
    y1_obs = 4;
    y2_obs = -4;
    
    dist_obs = 1.7;
    speed_obs = 0.75;
    
    dist_dt = speed_obs*scenario.dt; % distance traveled per timestep
    width_obs = 0.5;
    length_obs = 0.5;
    length_coll_area = length_obs+dist_dt;
    
    scenario.dynamic_obstacle_shape = [width_obs;length_obs];
    
    
    for iObstacle = 1:20
        for iTimestep = 1:80
            scenario.dynamic_obstacle_area{iObstacle,iTimestep} = transformedRectangle(...
                x1_obs ...
                ,center_y+y1_obs + iTimestep*dist_dt - iObstacle*dist_obs ...
                ,pi/2 ...
                ,length_coll_area ...
                ,width_obs ...
            );
            scenario.dynamic_obstacle_fullres{iObstacle,iTimestep} = [  ones(41,1)*x1_obs, ...
                                                        linspace(   center_y+y1_obs + iTimestep*dist_dt - iObstacle*dist_obs - dist_dt/2, ...
                                                                    center_y+y1_obs + (iTimestep+1)*dist_dt - iObstacle*dist_obs - dist_dt/2, 41)'];
                                                                
           scenario.dynamic_obstacle_area{iObstacle+20,iTimestep} = transformedRectangle(...
                x2_obs ...
                ,center_y+y2_obs - iTimestep*dist_dt + iObstacle*dist_obs ...
                ,pi/2 ...
                ,length_coll_area ...
                ,width_obs ...
            );
            scenario.dynamic_obstacle_fullres{iObstacle+20,iTimestep} = [  ones(41,1)*x2_obs, ...
                                                        linspace(   center_y+y2_obs - iTimestep*dist_dt + iObstacle*dist_obs + dist_dt/2, ...
                                                                    center_y+y2_obs - (iTimestep+1)*dist_dt + iObstacle*dist_obs + dist_dt/2, 41)'];
        end
    end
    
    scenario.name = sprintf('moving_obstacles');
end