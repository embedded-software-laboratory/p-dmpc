classdef Scenario
    % SCENARIO  Scenario class

    properties (Access = public)
        model = []; % instance of MuCar, BicycleModel or VehicleModel
        vehicles = []; % array of Vehicle objects
        obstacles = {}; % (n_obstacle, 1) static obstacles = {[x;y];...}
        dynamic_obstacle_area = {}; % (n_obstacle, Hp) dynamic obstacles = {[x;y],...}

        lanelets; % coordinates of all lanelets
        intersection_lanelets; % IDs of intersection lanelets
        road_raw_data; % raw road data
        road_data_file_path; % path to file of road data
        lanelet_boundary; % boundaries of all lanelets
        lanelet_relationships; % relationship between two adjacent lanelets
        adjacency_lanelets; % (nLanelets x nLanelets) matrix, entry is 1 if two lanelets are adjacent
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

            colors = rwth_color_order();

            for iVeh = 1:numel(obj.vehicles)
                veh = obj.vehicles(iVeh);
                % reference trajectory
                line( ...
                    veh.reference_path(:, 1), ...
                    veh.reference_path(:, 2), ...
                    'LineStyle', '--' ...
                );

                % vehicle rectangle
                veh.plot(colors(iVeh, :));
            end

            % Obstacles
            for o = obj.obstacles
                oCont = o{:};
                patch(oCont(1, :), oCont(2, :), [0.5 0.5 0.5]);
            end

            % lanelets
            if ~isempty(obj.road_raw_data) && ~isempty(obj.road_raw_data.lanelet)
                plot_lanelets(obj.road_raw_data.lanelet, options.scenario_type);
            end

            xlabel('$x$ [m]')
            ylabel('$y$ [m]')
        end

    end

end
