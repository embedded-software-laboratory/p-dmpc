function plot_runtime(data)
%   PLOT_RUNTIME     Export plots for runtime evaluation using pre-calculated data

    x_values = data.x_values;
    t_deadlock_free = data.t_deadlock_free;
    n_deadlock_free = data.n_deadlock_free;
    result = data.result;
    nVeh = data.nVeh;
    nPri = data.nPri;
    nSce = data.nSce;

    t_total = result.t_total;
    n_total = result.nSteps;

    n_x_values = length(x_values);
    
    t_total_vect = t_total*ones(1,n_x_values);

    total_nSce_per_Pri = nVeh*nSce;

    markers = {'x', '+', 'o', 'd'};

figHandle = figure();
tiledLayoutHandle = tiledlayout(3,1,'TileSpacing','Compact');
nexttile
hold on
plot(x_values,t_total_vect,'--')
% maximum values
for iPri = 1:nPri
    plot(x_values, max(t_deadlock_free(:,:,iPri),[],2),markers{iPri});
end
hold off
ylabel('$t_{\mathrm{max}}$','Interpreter','latex');

nexttile
hold on
plot(x_values,t_total_vect,'--')
% mean values
for iPri = 1:nPri
    plot(x_values, mean(t_deadlock_free(:,:,iPri),2),markers{iPri});
end
hold off
ylabel('$t_{\mathrm{mean}}$','Interpreter','latex');

nexttile
hold on
plot(x_values,t_total_vect,'--')
% minimum values
for iPri = 1:nPri
    plot(x_values, min(t_deadlock_free(:,:,iPri),[],2),markers{iPri});
end
hold off
ylabel('$t_{\mathrm{min}}$','Interpreter','latex');

legend( ...
    '$t_{\mathrm{exp}}$', ...
    '$p_{\mathrm{fca}}$', ...
    '$p_{rand}$', ...
    '$p_{\mathrm{const}}$', ...
    '$p_{\mathrm{color}}$', ...
    'Orientation','horizontal', ...
    'Location','southoutside', ...
    'Interpreter', 'latex' ...
);
ylabel(tiledLayoutHandle, 't until deadlock [s]','Interpreter','latex');
xlabel(tiledLayoutHandle, '\# of vehicles','Interpreter','latex');
xlim([6, 20])
% title(tiledLayoutHandle, 'Deadlock-Free Runtime');


% Export
folder_path = FileNameConstructor.gen_results_folder_path(result.scenario.options);
filename = 'deadlock-free-runtime-detail.pdf';
set_figure_properties(figHandle,'preset','document','paperheight_in',14)
export_fig(figHandle, fullfile(folder_path,filename));
close(figHandle);



% Overview
is_deadlock_free = (n_deadlock_free == n_total);
max_deadlock_free_vehicles = zeros(nPri,1);
mean_deadlock_free_vehicles = zeros(nPri,1);
min_deadlocked_vehicles = zeros(nPri,1);
n_dealock_free_scenarios = zeros(nPri,1);
vehicles_sce_pri = repmat((1:20)',1,nSce);

for iPri = 1:nPri
    n_vehicles_deadlock_free = vehicles_sce_pri(is_deadlock_free(:,:,iPri));
    n_dealock_free_scenarios(iPri) = sum(is_deadlock_free(:,:,iPri),'all');
    mean_deadlock_free_vehicles(iPri) = mean(n_vehicles_deadlock_free);
    max_deadlock_free_vehicles(iPri) = max(n_vehicles_deadlock_free);
    n_vehicles_deadlocked = vehicles_sce_pri(~is_deadlock_free(:,:,iPri));
    min_deadlocked_vehicles(iPri) = min(n_vehicles_deadlocked);
end

perc_deadlock_free_scenarios = n_dealock_free_scenarios./total_nSce_per_Pri;

bar_data = [...
    min_deadlocked_vehicles, ...
    mean_deadlock_free_vehicles, ...
    max_deadlock_free_vehicles ...
];

% plot
figHandle = figure();
% sort data
data_permutation = [1 4 2 3];
bar_data = bar_data(data_permutation,:);
barh(1:nPri, bar_data);
set(gca,'Ydir','reverse');
y_axis_handle = get(gca, 'YAxis');
y_axis_handle.TickLabelInterpreter = 'latex';

legend( ...
    'deadlock with least vehicles', ...
    'deadlock-free median', ...
    'deadlock-free maximum', ...
    'Location','southoutside' ...
);
legend('boxoff')

xlabel('$N_A$');
ylabel('Priority Assignment Function');
yticklabels({ ...
    '$p_{\mathrm{fca}}$', ...
    '$p_{\mathrm{color}}$', ...
    '$p_{\mathrm{rand}}$', ...
    '$p_{\mathrm{const}}$' ...
});


% Export
folder_path = FileNameConstructor.gen_results_folder_path( ...
    result.scenario.options ...
);
filename = 'deadlock-free-runtime-overview.pdf';
set_figure_properties(figHandle,'preset','paper','paperheight_in',8)
export_fig(figHandle, fullfile(folder_path,filename));
close(figHandle);

% plot
figHandle = figure();
% sort data
data_permutation = [1 4 2 3];
bar_data = min_deadlocked_vehicles(data_permutation,:);
b = barh(1:nPri, bar_data);
set(gca,'Ydir','reverse');
y_axis_handle = get(gca, 'YAxis');
y_axis_handle.TickLabelInterpreter = 'latex';

xlabel('$N_A$');
% title('Least $N_A$ producing deadlock')
yticklabels({ ...
    '$p_{\mathrm{fca}}$', ...
    '$p_{\mathrm{color}}$', ...
    '$p_{\mathrm{rand}}$', ...
    '$p_{\mathrm{const}}$' ...
});

xlim([0 max(min_deadlocked_vehicles)+1])

b.FaceColor = 'flat';
rwth100 = rwth_color_order();
b.CData = rwth100(1:nPri,:);


% Export
folder_path = FileNameConstructor.gen_results_folder_path( ...
    result.scenario.options ...
);
filename = 'deadlock-free-runtime-least-veh-deadlock.pdf';
set_figure_properties(figHandle,'preset','paper')
export_fig(figHandle, fullfile(folder_path,filename));
close(figHandle);

% plot
figHandle = figure();
% sort data
data_permutation = [1 4 2 3];
bar_data = perc_deadlock_free_scenarios(data_permutation,:);
b = barh(1:nPri, bar_data);
set(gca,'Ydir','reverse');
y_axis_handle = get(gca, 'YAxis');
y_axis_handle.TickLabelInterpreter = 'latex';

xlabel('Deadlock free from all scenarios [\%]');
yticklabels({ ...
    '$p_{\mathrm{fca}}$', ...
    '$p_{\mathrm{color}}$', ...
    '$p_{\mathrm{rand}}$', ...
    '$p_{\mathrm{const}}$' ...
});

b.FaceColor = 'flat';
rwth100 = rwth_color_order();
b.CData = rwth100(1:nPri,:);


% Export
folder_path = FileNameConstructor.gen_results_folder_path( ...
    result.scenario.options ...
);
filename = 'deadlock-free-runtime-percentage-deadlock-free-scenarios.pdf';
set_figure_properties(figHandle,'preset','paper')
export_fig(figHandle, fullfile(folder_path,filename));
close(figHandle);
end
