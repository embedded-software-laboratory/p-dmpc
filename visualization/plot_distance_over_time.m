function plot_distance_over_time(result)
% PLOT_DISTANCE_OVER_TIME   Plot distance over time

fig = figure('Visible','off');

T_end = 2.8;
for ir = 1:numel(result)
    r = result(ir);
    nSteps = min(size(r.vehicle_path_fullres,2), round(T_end/r.scenario.dt));
    for iVeh = 1:numel(r.scenario.vehicles)
        % Could also integrate speed, here sqrt of distance
        x = zeros(nSteps*r.scenario.tick_per_step+1,1);
        y = zeros(nSteps*r.scenario.tick_per_step+1,1);
        for j = 1:nSteps
            x(1+(j-1)*r.scenario.tick_per_step:j*r.scenario.tick_per_step) = ...
                r.vehicle_path_fullres{iVeh,j}(1:r.scenario.tick_per_step,1);
            y(1+(j-1)*r.scenario.tick_per_step:j*r.scenario.tick_per_step) = ...
                r.vehicle_path_fullres{iVeh,j}(1:r.scenario.tick_per_step,2);
        end
        x(end) = r.vehicle_path_fullres{iVeh,nSteps}(end,1);
        y(end) = r.vehicle_path_fullres{iVeh,nSteps}(end,2);
        dx = diff(x);
        dy = diff(y);
        ds = sqrt(dx.^2 + dy.^2);
        s = zeros(nSteps*r.scenario.tick_per_step+1,1);
        s(2:end) = cumsum(ds);
        t = r.scenario.time_per_tick*(0:nSteps*r.scenario.tick_per_step);
        % reference trajectory
        if ir ~=3
            linestyle = '-';
            c = vehColor(ir);
        else
            linestyle = ':';
            c = vehColor(2);
        end
        line( t, s, 'Color',c, 'LineStyle', linestyle );
    end
end

r = result(1);

xticks(0:r.scenario.dt:T_end);
grid on;
xlim([0, T_end]);
% Plot obstacle
t = r.scenario.time_per_tick*(0:T_end/r.scenario.dt*r.scenario.tick_per_step);
x_max = r.scenario.obstacles{1}(1,1)...
        - r.scenario.vehicles(1).Length/2 ...
        - r.scenario.offset...
        - r.scenario.vehicles(1).x_start;
line( t, x_max*ones(size(t)), 'Color', [0.5 0.5 0.5] );

% labels
xlabel("Time [s]");
ylabel("Distance [m]");

% export
filepath = fullfile(fileparts(r.output_path), 'recursive_feasibility_t-d.pdf');
set_figure_properties(fig,'paper');
exportgraphics(fig, filepath, 'ContentType','vector');
close(fig);
end