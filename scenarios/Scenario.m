classdef Scenario
    properties
        vehicles = [];  % array of Vehicle objects
        obstacles = {}; % obstacles = {[xs;ys],...}
        nVeh = 0;
        name = 'UnnamedScenario';
        controller_name = 'RHC';
        dt = 0.4;     % RHC sample time [s]
        Hp = 5;
        Hu;
        mpa;
        trim_set = 1;
        offset = 0.03;  % offset for collision checks
        model = [];
        time_per_tick = 0.01;
        tick_per_step;
        r_goal = 0.1;   % goal circle
    end
    
    methods
        function obj = Scenario(options)
            obj.Hu = obj.Hp;
            obj.tick_per_step = obj.dt/obj.time_per_tick;
            if nargin == 1
                radius = 2;
                for the_angle=options.angles
                    s = sin(the_angle);
                    c = cos(the_angle);
                    veh = Vehicle();
                    veh.x_start = -c*radius;
                    veh.y_start = -s*radius;
                    veh.yaw_start = the_angle;
                    veh.x_goal = c*radius;
                    veh.y_goal = s*radius;
                    veh.yaw_goal = the_angle;
                    veh.trim_config = 1;
                    veh.referenceTrajectory = [-c*radius -s*radius;c*radius s*radius];
                    
                    % Lab: translate by center
                    center_x = 2.25;
                    center_y = 2;
                    veh.x_start = -c*radius + center_x;
                    veh.y_start = -s*radius + center_y;
                    veh.x_goal = c*radius + center_x;
                    veh.y_goal = s*radius + center_y;
                    veh.referenceTrajectory = veh.referenceTrajectory + [center_x, center_y];
                    obj.vehicles = [obj.vehicles, veh];
                end
                
                obj.nVeh = options.amount;
                obj.name = sprintf('%i-circle', options.amount);

                obj.model = BicycleModel(veh.Lf,veh.Lr);

                recursive_feasibility = true;
                obj.mpa = MotionPrimitiveAutomaton(...
                    obj.model...
                    , obj.trim_set...
                    , obj.offset...
                    , obj.dt...
                    , options.amount...
                    , obj.Hp...
                    , obj.tick_per_step...
                    , recursive_feasibility...
                );
            end
        end
        
        function plot(obj)
            for iVeh = 1:numel(obj.vehicles)
                % vehicle rectangle
                veh = obj.vehicles(iVeh);
                veh.plot(vehColor(iVeh));
                % reference trajectory
                line(   veh.referenceTrajectory(:,1), ...
                        veh.referenceTrajectory(:,2), ...
                        'Color',vehColor(iVeh),'LineStyle', '--', 'LineWidth',1 ...
                );
            end
        end
    end
    
    
end