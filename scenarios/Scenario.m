classdef Scenario
% SCENARIO  Scenario class    

    properties
        vehicles = [];  % array of Vehicle objects
        obstacles = {}; % obstacles = {[xs;ys],...}
        nVeh = 0;
        name = 'UnnamedScenario';
        controller_name = 'RHC';
        controller = @(s,i) centralized_controller(s,i);
        dt = 0.4;     % RHC sample time [s]
        T_end = 20;   % Duration of simulation. [s]
        Hp = 5;
        mpa;
        trim_set = 3;
        offset_length = 0.01;  % offset for collision checks
        offset_width = 0.01;  % offset for collision checks
        
        model = [];
        time_per_tick = 0.01;
        r_goal = 0.1;   % goal circle
        dynamic_obstacle_area;
        dynamic_obstacle_shape;
        dynamic_obstacle_fullres;
        plot_limits = [-10,10;-10,10]; % default fallback if not defined
        adjacency;
        directed_coupling;
        assignPrios = false;
        vehicle_to_lanelet
        lanelets
        intersection_lanelets
        boundary
        commonroad_data
    end
    
    properties (Dependent)
        tick_per_step;
        Hu;
        k_end;
    end
    
    methods
        function obj = Scenario()
        end
        
        function result = get.k_end(obj)
            result = floor(obj.T_end/obj.dt);
        end
        function result = get.tick_per_step(obj)
            result = obj.dt/obj.time_per_tick;
        end
        function result = get.Hu(obj)
            result = obj.Hp;
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
