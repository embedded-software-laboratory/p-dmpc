classdef Scenario
    % SCENARIO  Scenario class

    properties (Access = public)
        vehicles (1, :) Vehicle; % array of Vehicle objects
        obstacles = {}; % (n_obstacle, 1) static obstacles = {[x;y];...}
        dynamic_obstacle_area = {}; % (n_obstacle, Hp) dynamic obstacles = {[x;y],...}

        lanelets; % coordinates of all lanelets
        intersection_lanelets; % IDs of intersection lanelets
        road_raw_data; % raw road data
        road_data_file_path; % path to file of road data
        lanelet_boundary; % boundaries of all lanelets
        lanelet_relationships; % relationship between two adjacent lanelets
        adjacency_lanelets (:, :) logical; % (nLanelets x nLanelets) matrix, entry is 1 if two lanelets are adjacent
        intersection_center = [2.25, 2]; % (numOfIntersection x 2) matrix, positions of intersection center

        plot_limits (2, 2) double = [0, 4.5; 0, 4]; % [xmin xmax; ymin ymax]
    end

    methods

        function obj = Scenario(options)
            obj.vehicles(1, options.amount) = Vehicle();
        end

        function plot(obj, optional)

            arguments
                obj (1, 1) Scenario;
                optional.fig (1, 1) matlab.ui.Figure = figure(Visible = "on");
            end

            set(0, 'CurrentFigure', optional.fig);
            daspect([1 1 1]);

            xlim(obj.plot_limits(1, :));
            ylim(obj.plot_limits(2, :));

            for iVeh = 1:numel(obj.vehicles)
                veh = obj.vehicles(iVeh);
                % reference trajectory
                line( ...
                    veh.reference_path(:, 1), ...
                    veh.reference_path(:, 2), ...
                    LineStyle = '--', ...
                    Color = rwth_color_order(iVeh) ...
                );

                % vehicle rectangle
                veh.plot(rwth_color_order(iVeh));

                % path ID
                text( ...
                    veh.x_start, ...
                    veh.y_start, ...
                    num2str(iVeh) ...
                );
            end

            % Obstacles
            for o = obj.obstacles
                oCont = o{:};
                patch(oCont(1, :), oCont(2, :), [0.5 0.5 0.5]);
            end

            xlabel('$x$ [m]')
            ylabel('$y$ [m]')
        end

    end

    methods (Static)

        function scenario = create(options)

            arguments
                options (1, 1) Config;
            end

            switch options.scenario_type
                case ScenarioType.circle
                    scenario = Circle(options);
                case ScenarioType.commonroad
                    scenario = Commonroad(options);
            end

        end

    end

end
