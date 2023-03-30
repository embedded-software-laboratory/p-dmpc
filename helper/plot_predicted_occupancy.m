function plot_predicted_occupancy(trajectory_predictions,scenario,color,trims_stop,is_one_step_shifted)
% PLOT_PREDICTED_OCCUPANCY Plot predicted occupied areas
% Input
%   trajectory_predictions: predicted trajectory
%   scenario: instance of the class "Scenario"
%   color: define color of the area
%   trims_step: equlibrium trim(s)
%   is_one_step_shifted: true/false, if true, one-step-shifted occupancy
%   will be shown
%   
    n_predictions = size(trajectory_predictions, 1);
    n_ticks = n_predictions / scenario.options.Hp;

    i = 0;
    for tick = n_predictions-n_ticks+1:-n_ticks:1
        if is_one_step_shifted && tick == 1
            % ignore the first timestep
            continue
        end
        i = i + 1;
        x = trajectory_predictions(tick, :);
        trim1 = trajectory_predictions(tick, 4);
        if i == 1
            assert(length(trims_stop)==1)
            trim2 = trims_stop; % because of recursive feasibility, the last trim must be the equilibrium trim (speed=0)
        else
            trim2 = trajectory_predictions(tick + n_ticks, 4);
        end
        area = scenario.mpa.maneuvers{trim1, trim2}.area;
        [area_x, area_y] = translate_global(x(3), x(1), x(2), area(1, :), area(2, :));
        area_poly = polyshape([area_x; area_y]');
        plot(area_poly, 'FaceColor', color, 'FaceAlpha', (i/(scenario.options.Hp+2))*0.5)
    end
end