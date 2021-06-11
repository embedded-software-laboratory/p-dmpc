function plot_distance_over_time(result)
fig = figure('Visible','on');

for ir = 1:numel(result)
    r = result(ir);
    nSteps = size(r.vehicle_path_fullres,2);
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
        x(end) = r.vehicle_path_fullres{iVeh,end}(end,1);
        y(end) = r.vehicle_path_fullres{iVeh,end}(end,2);
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

% Plot obstacle
t = result(1).scenario.time_per_tick*(0:size(result(1).vehicle_path_fullres,2)*result(1).scenario.tick_per_step);
x_max = result(1).scenario.obstacles{1}(1,1)...
        - result(1).scenario.vehicles(1).Length/2 ...
        - result(1).scenario.offset...
        - result(1).scenario.vehicles(1).x_start;
line( t, x_max*ones(size(t)), 'Color', [0.5 0.5 0.5] );

% labels
xlabel("Time [s]");
ylabel("Distance [m]");

% export
filetype = 'pdf';
filepath = fullfile(result(1).output_path, ['t-d.' filetype]);
set_figure_properties(fig,'paper');
exportgraphics(fig, filepath, 'ContentType','vector');
close(fig);
end