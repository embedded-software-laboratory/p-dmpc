function [ res ] = eval_plot_runtime(res)
% EVAL_PLOT_RUNTIME Evaluate the runtime of the experiment before deadlock

[nVeh, nPri, nSce ] = size(res);

x_values = zeros(1, nVeh);
t_deadlock_free = zeros(nVeh, nSce, nPri);
for iVeh = 1:nVeh
    x_values(iVeh) = res{iVeh,1,1}.scenario.options.amount;
    for iPri = 1:nPri
        for iSce = 1:nSce
            result = res{iVeh,iPri,iSce};
            [~,t_total_deadlock_free] = compute_deadlock_free_runtime(result);
            t_deadlock_free(iVeh, iSce, iPri) = t_total_deadlock_free;
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
% title(tiledLayoutHandle, 'Deadlock-Free Runtime');


% Export
folder_path = FileNameConstructor.gen_results_folder_path(res{1,1,1}.scenario.options);
filename = 'deadlock-free-runtime.pdf';
set_figure_properties(figHandle,'document',14);
export_fig(figHandle, fullfile(folder_path,filename));
close(figHandle);
end