function plot_scenario(result)
fig = figure('Visible','off');
daspect([1 1 1]);

xlim(result.plot_limits(1,:));
ylim(result.plot_limits(2,:));

% reference trajectory
for iVeh = 1:numel(result.scenario.vehicles)
    veh = result.scenario.vehicles(iVeh);
    line(   veh.referenceTrajectory(:,1), ...
            veh.referenceTrajectory(:,2), ...
            'Color',vehColor(iVeh),'LineStyle', '--', 'LineWidth',1 ...
    );
end

% vehicle rectangle
for iVeh = 1:numel(result.scenario.vehicles)
    veh = result.scenario.vehicles(iVeh);
    veh.plot(vehColor(iVeh));
end

% Obstacles
for o = result.scenario.obstacles
    oCont = o{:};
    patch(oCont(1,:),oCont(2,:),[0.5 0.5 0.5]);
end

xlabel('$x$ [m]')
ylabel('$y$ [m]')

filetype = 'pdf';
filepath = fullfile(result.output_path, ['scenario.' filetype]);
set_figure_properties(fig,'paper')
exportgraphics(fig, filepath, 'ContentType','vector');
% filepath = fullfile(result.output_path, 'scenario');
% export_fig(filepath, '-pdf','-painters');
close(fig);
end