function runtimePlotCentralized( results )
%RUNTIMEPLOT    Creates boxplots for result-struct-vector visualizing computation time in centralizied RHC

close all;
nResults = length(results);

fig = figure('visible','off','position',[100 100 600 630],'color',[1 1 1]);

runtimes = [];
nVehicles = [];

for i = 1 : nResults
    % make sure only one controller runtime is stored in the struct
    assert(size(results(i).controller_runtime, 1) == 1);
    runtimes = [runtimes, results(i).controller_runtime'];
    nVehicles = [nVehicles, results(i).scenario.nVeh];
end

[~,order] = sort(nVehicles);

runtimes(:,:) = runtimes(:,order);
nVehicles(:) = nVehicles(order);

% y axis on left and right side
ax = axes();
yyaxis right;
linkprop(ax.YAxis, 'Limits');
set(gca,'ycolor','black');
yyaxis left;
set(gca,'ycolor','black');

% plot
boxplot(runtimes,nVehicles,'MedianStyle','line');

% TODO: plot limits - outlier

% set labels
xlabel('Number of Vehicles','Interpreter','LaTex');
ylabel('Runtime [s]','Interpreter','LaTex');
title('Controller runtime in Circle scenario','Interpreter','LaTex');

set_figure_properties(fig, 'paper', 12);
filepath = fullfile('results', 'runtimePlot.pdf');
exportgraphics(fig, filepath, 'ContentType','vector');
close(fig);

end

