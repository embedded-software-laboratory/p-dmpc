function computation_time_plot( results )
%COMPUTATION_TIME_PLOT    Creates boxplots for result-struct-vector visualizing computation time in centralizied RHC

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

% plot with axis on both sides and logarithmic scale
colororder({'k','k'});
yyaxis right;
boxplot(runtimes,nVehicles,'MedianStyle','line');
set(gca, 'YScale', 'log');
yyaxis left;
boxplot(runtimes,nVehicles,'MedianStyle','line');
set(gca, 'YScale', 'log');

% set labels
xlabel('Number of Vehicles','Interpreter','LaTex');
ylabel('Computation Time [s]','Interpreter','LaTex');

set_figure_properties(fig, 'paper', 12);
filepath = fullfile('results', 'computation_time.pdf');
exportgraphics(fig, filepath, 'ContentType','vector');
close(fig);

end

