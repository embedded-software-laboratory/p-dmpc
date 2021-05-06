classdef Scenario
    properties
        vehicles = [];  % array of Vehicle objects
        obstacles = {}; % obstacles = {[xs;ys],...}
        nVeh = 0;
        name = 'UnnamedScenario';
        dt;             % RHC sample time [s]
        Hp;
        Hu;
        mpa;
        trim_set = 3;
        offset = 0.03;  % offset for collision checks
        model = [];
        r_goal = 0.1;   % goal circle
    end
    
    methods
        function obj = Scenario(options)
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
            obj.name = sprintf("%i-circle", options.amount);
            obj.Hp = 5;
            obj.Hu = obj.Hp;
            obj.dt = 0.4;

%             obj.obstacles{1} = [-0.1  0.1  0.1 -0.1
%                                 -0.2 -0.2  0.2  0.2] ...
%                              + [2; 2];

            veh = Vehicle();
            obj.model = BicycleModel(veh.Lf,veh.Lr);

            obj.mpa = MotionPrimitiveAutomaton(obj.model, obj.trim_set, obj.offset, obj.dt, options.amount, obj.Hp);
        end
        
        function plot(obj)
            veh_colors = [  0.8941    0.1020    0.1098  ;...
                            0.2157    0.4941    0.7216  ;...
                            0.3020    0.6863    0.2902  ;...
                            0.5961    0.3059    0.6392  ;...
                            1.0000    0.4980    0       ;...
                            1.0000    1.0000    0.2000  ;...
                            0.6510    0.3373    0.1569  ;...
                            0.9686    0.5059    0.7490  ];
            for iVeh = 1:numel(obj.vehicles)
                % vehicle rectangle
                veh = obj.vehicles(iVeh);
                veh.plot(veh_colors(mod(iVeh-1,size(veh_colors,1))+1,:));
            end
        end
    end
    
    
end