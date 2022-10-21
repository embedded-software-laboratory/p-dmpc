function eval_plot_runtime(res)
% EVAL_PLOT_RUNTIME Evaluate the runtime of the experiment before deadlock

[nVeh, nPri, nSce ] = size(res);

x_values = zeros(1, nVeh);
t_deadlock_free = zeros(nVeh, nSce, nPri);
n_deadlock_free = zeros(nVeh, nSce, nPri);
for iVeh = 1:nVeh
    x_values(iVeh) = res{iVeh,1,1}.scenario.options.amount;
    for iPri = 1:nPri
        for iSce = 1:nSce
            result = res{iVeh,iPri,iSce};
            [n_total_deadlock_free,t_total_deadlock_free] = compute_deadlock_free_runtime(result);
            t_deadlock_free(iVeh, iSce, iPri) = t_total_deadlock_free;
            n_deadlock_free(iVeh, iSce, iPri) = n_total_deadlock_free;
        end
    end
end

markers = {'x', '+', 'o', 'd'};

resstruct=[res{:,1,1}]; % assumes equal duration of scenarios

figHandle = figure();
tiledLayoutHandle = tiledlayout(3,1,'TileSpacing','Compact');
nexttile
hold on
plot(x_values,[resstruct.t_total],'--')
% maximum values
for iPri = 1:nPri
    plot(x_values, max(t_deadlock_free(:,:,iPri),[],2),markers{iPri});
end
hold off
ylabel('$t_{\mathrm{max}}$','Interpreter','latex');

nexttile
hold on
plot(x_values,[resstruct.t_total],'--')
% mean values
for iPri = 1:nPri
    plot(x_values, mean(t_deadlock_free(:,:,iPri),2),markers{iPri});
end
hold off
ylabel('$t_{\mathrm{mean}}$','Interpreter','latex');

nexttile
hold on
plot(x_values,[resstruct.t_total],'--')
% minimum values
for iPri = 1:nPri
    plot(x_values, min(t_deadlock_free(:,:,iPri),[],2),markers{iPri});
end
hold off
ylabel('$t_{\mathrm{min}}$','Interpreter','latex');

legend( ...
    '$t_{\mathrm{exp}}$', ...
    '$p_{\mathrm{fca}}$', ...
    '$p_{r}$', ...
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
folder_path = FileNameConstructor.gen_results_folder_path(res{1,1,1}.scenario.options);
filename = 'deadlock-free-runtime-detail.pdf';
set_figure_properties(figHandle,'preset','document','paperheight_in',14)
export_fig(figHandle, fullfile(folder_path,filename));
close(figHandle);



% Overview
n_total = res{1,1,1}.nSteps;
is_deadlock_free = (n_deadlock_free == n_total);
max_deadlock_free_vehicles = zeros(nPri,1);
mean_deadlock_free_vehicles = zeros(nPri,1);
min_deadlocked_vehicles = zeros(nPri,1);
vehicles_sce_pri = repmat((1:20)',1,nSce);

for iPri = 1:nPri
    n_vehicles_deadlock_free = vehicles_sce_pri(is_deadlock_free(:,:,iPri));
    mean_deadlock_free_vehicles(iPri) = mean(n_vehicles_deadlock_free);
    max_deadlock_free_vehicles(iPri) = max(n_vehicles_deadlock_free);
    n_vehicles_deadlocked = vehicles_sce_pri(~is_deadlock_free(:,:,iPri));
    min_deadlocked_vehicles(iPri) = min(n_vehicles_deadlocked);
end
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
    res{1,1,1}.scenario.options ...
);
filename = 'deadlock-free-runtime-overview.pdf';
set_figure_properties(figHandle,'preset','paper','paperheight_in',8)
export_fig(figHandle, fullfile(folder_path,filename));
close(figHandle);
end