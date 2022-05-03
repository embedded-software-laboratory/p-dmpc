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
        T_end = 18;   % Duration of simulation. [s]
        Hp = 4;
        mpa;
        trim_set = 3;
        offset = 0.01;
        
        model = [];
        time_per_tick = 0.01;
        r_goal = 0.1;   % goal circle
        dynamic_obstacle_area;
        dynamic_obstacle_shape;
        dynamic_obstacle_fullres;
        dynamic_obstacle_reachableSets; % reachable sets of the coupled vehicles with higher priorities in other groups
        plot_limits = [-10,10;-10,10]; % default fallback if not defined
        adjacency;
        semi_adjacency;
        directed_coupling;
        assignPrios = false;
        isParl = false % if use parallel computation (logical variable)
        communication = {} % all the information related to communication between vehicles
%         road_info = struct % road information including all lanelets and position of all vehicles
        groups_info = {} % groups information about which vehicles form a group and which vehicles have which priorities
        vehicle_to_lanelet;
        lanelets; % position of all lanelets
        intersection_lanelets; % indices of intersection lanelets
        boundary;
        commonroad_data; % metadata
        lanelet_boundary; % boundaries of all lanelets
        last_veh_at_intersection = []; % store information about which vehicles were at the intersection in the last time step
        k; %simulation steps
        priority_option;
        is_single_lane = false; % If true, vehicles are not allowed to switch to the adjacent parallel lane
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
