classdef Scenario < handle
    properties
        vehicles = [];  % vehicles
        nVeh = 0;
        name = 'UnnamedScenario';
        dt   = 0.4;     % RHC sample time [s]
        Hp   = 6;
        Hu   = 6;
        combined_graph;
        trim_set = 'trim_set_3_1';
    end
    
    methods
        function obj = Scenario(options)
            radius = 15;
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
                obj.vehicles = [obj.vehicles, veh];
            end
            obj.nVeh = options.amount;
            obj.name = sprintf("%i-circle", options.amount);
            obj.Hp = h_p;
            obj.Hu = h_u;


            
            % Make motionGraph Tupel
            motionGraphList = create_motion_graph_list(obj.trim_set, options.amount);
            % Combine graphs
            obj.combined_graph = CombinedGraph(motionGraphList);
        end
        
        function plot(obj, varargin)
            if nargin==1
                fig = figure;
            else
                fig = varargin{1};
            end
            figure(fig);
            veh_colors = [  0.8941    0.1020    0.1098  ;...
                            0.2157    0.4941    0.7216  ;...
                            0.3020    0.6863    0.2902  ;...
                            0.5961    0.3059    0.6392  ;...
                            1.0000    0.4980    0       ;...
                            1.0000    1.0000    0.2000  ;...
                            0.6510    0.3373    0.1569  ;...
                            0.9686    0.5059    0.7490  ];
            for iVeh = 1:numel(obj.vehicles)
                hold on
                % vehicle rectangle
                veh = obj.vehicles(iVeh);
                veh.plot(fig, veh_colors(mod(iVeh-1,size(veh_colors,1))+1,:));
                hold off
            end
            axis equal
        end
    end
    
    
end