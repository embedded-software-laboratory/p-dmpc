classdef Scenario
    % SCENARIO  Scenario class

    properties (Access = public)
        vehicles = []; % array of Vehicle objects
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
    end

    methods

        function obj = Scenario()
        end

        function plot(obj, options, optional)

            arguments
                obj (1, 1) Scenario;
                options (1, 1) Config;
                optional.fig (1, 1) matlab.ui.Figure = figure(Visible = "on");
            end

            set(0, 'CurrentFigure', optional.fig);
            daspect([1 1 1]);

            xlim(options.plot_limits(1, :));
            ylim(options.plot_limits(2, :));

            for iVeh = 1:numel(obj.vehicles)
                veh = obj.vehicles(iVeh);
                % reference trajectory
                line( ...
                    veh.reference_path(:, 1), ...
                    veh.reference_path(:, 2), ...
                    LineStyle = '--' ...
                );

                % vehicle rectangle
                veh.plot(rwth_color_order(iVeh));
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
                    scenario = Circle(options.amount);
                case ScenarioType.commonroad
                    scenario = Commonroad(options.amount, options.path_ids);
                case ScenarioType.lanelet2
                    scenario = lanelet2_scenario(options.amount, options.path_ids);
                case ScenarioType.lab_default
                    scenario = Scenario();
                    disp('Scenario is created later after map is retrieved via UnifiedLabApi');
            end

        end

    end

end
