function runtimePlotCentralized( results )
%RUNTIMEPLOT    Creates boxplots for result-struct-vector visualizing computation time in centralizied RHC

close all;
nResults = length(results);

fig = figure('visible','off','position',[100 100 600 630],'color',[1 1 1]);

runtimes = [];
labels = {};

for i = 1 : nResults
    % make sure only one controller runtime is stored in the struct
    assert(size(results(i).controller_runtime, 1) == 1);
    runtimes = [runtimes, results(i).controller_runtime'];
    labels{end+1} = results(i).scenario.name;
end

% y axis on left and right side
ax = axes();
yyaxis right;
linkprop(ax.YAxis, 'Limits');
set(gca,'ycolor','black');
yyaxis left;
set(gca,'ycolor','black');

% plot
boxplot(runtimes,'Labels',labels,'MedianStyle','line');

% TODO: plot limits - outlier

% set labels
xlabel('Scenario','Interpreter','LaTex');
ylabel('Runtime [s]','Interpreter','LaTex');
title('Controller runtime','Interpreter','LaTex');

set_figure_properties(fig, 'paper', 12);
filepath = fullfile('results', 'runtimePlot.pdf');
exportgraphics(fig, filepath, 'ContentType','vector');
close(fig);

end

