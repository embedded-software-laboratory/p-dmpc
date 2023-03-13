function plot_scenario(result, filename)
% PLOT_SCENARIO     Plot scenario setup
arguments
    result (1,1)
    filename char = 'scenario.pdf'
end

scenario = result.scenario;

fig = figure('Visible','on');
daspect([1 1 1]);

xlim(scenario.options.plot_limits(1,:));
ylim(scenario.options.plot_limits(2,:));

% reference trajectory
for iVeh = 1:numel(scenario.vehicles)
    veh = scenario.vehicles(iVeh);
    line(   veh.referenceTrajectory(:,1), ...
            veh.referenceTrajectory(:,2), ...
            'Color',vehColor(iVeh),'LineStyle', '--', 'LineWidth',1 ...
    );
end

% vehicle rectangle
for iVeh = 1:numel(scenario.vehicles)
    veh = scenario.vehicles(iVeh);
    veh.plot(vehColor(iVeh));
end

% Obstacles
for o = result.obstacles
    oCont = o{:};
    patch(oCont(1,:),oCont(2,:),[0.5 0.5 0.5]);
end

% lanelets
if ~isempty(scenario.road_raw_data.lanelet)
    plot_lanelets(scenario.road_raw_data.lanelet,scenario.options.scenario_name);
end

xlabel('$x$ [m]')
ylabel('$y$ [m]')

folder_path = FileNameConstructor.gen_results_folder_path( ...
    scenario.options ...
);
set_figure_properties(fig,ExportFigConfig.paper("paperheight",6))

export_fig(fig,fullfile(folder_path,filename));
close(fig);
end
