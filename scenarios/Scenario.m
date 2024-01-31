classdef Scenario
    % SCENARIO  Scenario class

    properties (Access = public)
        vehicles = []; % array of Vehicle objects
        obstacles = {}; % (n_obstacle, 1) static obstacles = {[x;y];...}
        dynamic_obstacle_area = {}; % (n_obstacle, Hp) dynamic obstacles = {[x;y],...}
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

end
