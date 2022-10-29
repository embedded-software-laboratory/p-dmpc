function plot_runtime_data(data)
    % PLOT_RUNTIME_DATA     Export plots for runtime evaluation using pre-calculated data
    
    %% Prepare data
    x_values = data.x_values;
    t_deadlock_free = data.t_deadlock_free;
    n_deadlock_free = data.n_deadlock_free;
    avg_speed_by_veh_pri = data.avg_speed_by_veh_pri;
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
    
    data_permutation = [1 4 2 3];

    %% Plot: Deadlock-free runtime
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
    
    
    
    %% Plot 2: Standstill with least vehicles
    % TODO delete

    % figHandle = figure();
    % % sort data
    % data_permutation = [1 4 2 3];
    % bar_data = min_deadlocked_vehicles(data_permutation,:);
    % b = barh(1:nPri, bar_data);
    % set(gca,'Ydir','reverse');
    % y_axis_handle = get(gca, 'YAxis');
    % y_axis_handle.TickLabelInterpreter = 'latex';
    
    % xlabel('$N_A$');
    % title('Standstill with least vehicles')
    % yticklabels({ ...
    %     '$p_{\mathrm{fca}}$', ...
    %     '$p_{\mathrm{color}}$', ...
    %     '$p_{\mathrm{rand}}$', ...
    %     '$p_{\mathrm{const}}$' ...
    % });
    
    % xlim([0 max(min_deadlocked_vehicles)+1])
    
    % b.FaceColor = 'flat';
    % rwth100 = rwth_color_order();
    % b.CData = rwth100(1:nPri,:);
    
    
    % % Export
    % folder_path = FileNameConstructor.gen_results_folder_path( ...
    %     result.scenario.options ...
    % );
    % filename = 'deadlock-free-runtime-least-veh-deadlock.pdf';
    % set_figure_properties(figHandle,'preset','paper','paperheight_in',3)
    % export_fig(figHandle, fullfile(folder_path,filename));
    % close(figHandle);
    
    %% Plot 3: Percent standstill free scenarios
    % perc_deadlock_free_scenarios = n_deadlock_free_scenarios./total_nSce_per_Pri;
    % figHandle = figure();
    % % sort data
    % data_permutation = [1 4 2 3];
    % bar_data = perc_deadlock_free_scenarios(data_permutation,:);
    % b = barh(1:nPri, bar_data);
    % set(gca,'Ydir','reverse');
    % y_axis_handle = get(gca, 'YAxis');
    % y_axis_handle.TickLabelInterpreter = 'latex';
    
    % xlabel('Standstill-free scenarios [\%]');
    % yticklabels({ ...
    %     '$p_{\mathrm{fca}}$', ...
    %     '$p_{\mathrm{color}}$', ...
    %     '$p_{\mathrm{rand}}$', ...
    %     '$p_{\mathrm{const}}$' ...
    % });
    
    % b.FaceColor = 'flat';
    % rwth100 = rwth_color_order();
    % b.CData = rwth100(1:nPri,:);
    
    
    % % Export
    % folder_path = FileNameConstructor.gen_results_folder_path( ...
    %     result.scenario.options ...
    % );
    % filename = 'deadlock-free-runtime-percentage-deadlock-free-scenarios.pdf';
    % set_figure_properties(figHandle,'preset','paper','paperheight_in',3)
    % export_fig(figHandle, fullfile(folder_path,filename));
    % close(figHandle);
    


    %% Plot: average speed over nVeh
    figHandle = figure();
    
    p = plot(avg_speed_by_veh_pri(:,data_permutation),'-');
    [p.Marker] = markers{:};
    xticks(x_values)
    ylabel('Average speed [m/s]','Interpreter','latex');
    xlabel('$N_{\mathrm{A}}$','Interpreter','latex');
    legend( ...
        '$p_{\mathrm{fca}}$', ...
        '$p_{\mathrm{color}}$', ...
        '$p_{\mathrm{rand}}$', ...
        '$p_{\mathrm{const}}$', ...
        'Location','best' ...
    );

    % Export document presets
    folder_path = FileNameConstructor.gen_results_folder_path( ...
        result.scenario.options ...
    );
    filename = 'deadlock-free-avg-speed-over-nVeh.pdf';
    set_figure_properties(figHandle,'preset','document')
    export_fig(figHandle, fullfile(folder_path,filename));
    close(figHandle);
    


    %% Plot: matrix over criteria
    % min number vehicles for standstill
    is_deadlock_free = (n_deadlock_free == n_total);
    min_deadlocked_vehicles = zeros(nPri,1);
    n_deadlock_free_scenarios = zeros(nPri,1);
    vehicles_sce_pri = repmat((1:20)',1,nSce);
    
    for iPri = 1:nPri
        n_deadlock_free_scenarios(iPri) = sum(is_deadlock_free(:,:,iPri),'all');
        n_vehicles_deadlocked = vehicles_sce_pri(~is_deadlock_free(:,:,iPri));
        min_deadlocked_vehicles(iPri) = min(n_vehicles_deadlocked);
    end

    min_nVeh_standstill = min_deadlocked_vehicles(data_permutation,:)';

    % percentage deadlock free
    perc_deadlock_free_scenarios = n_deadlock_free_scenarios./total_nSce_per_Pri;
    % sort data
    perc_standstill_free = perc_deadlock_free_scenarios(data_permutation,:)';

    % time until standstill
    t_standstill_free = reshape( ...
        mean(t_deadlock_free,[1 2]), ...
        1, [] ...
    );
    
    % disp values
    fprintf("Min N_A ")
    fprintf(" %6.4g", min_nVeh_standstill)
    fprintf("\n")
    fprintf("%% Scen. ")
    fprintf(" %6.4g", perc_deadlock_free_scenarios)
    fprintf("\n")
    fprintf("Time    ")
    fprintf(" %6.4g", t_standstill_free)
    fprintf("\n")

    % plot
    scaled_nVeh = rescale(min_nVeh_standstill,'InputMin',0);
    scaled_t    = rescale(t_standstill_free, 'InputMin',0);
    criteria_mat = [
        scaled_nVeh
        perc_standstill_free
        scaled_t
    ];

    figHandle = figure();
    p = plot(criteria_mat,':');
    [p.Marker] = markers{:};
    xticks(1:3)
    xticklabels({'$N_A$ min', '\% succ.', 'Time'})
    ax = gca;
    ax.TickLabelInterpreter = 'latex';
    xlim([0.8, 3.2]);
    legend( ...
        '$p_{\mathrm{fca}}$', ...
        '$p_{\mathrm{color}}$', ...
        '$p_{\mathrm{rand}}$', ...
        '$p_{\mathrm{const}}$', ...
        'Location','southeast' ...
    );

    % Export document presets
    folder_path = FileNameConstructor.gen_results_folder_path( ...
        result.scenario.options ...
    );
    filename = 'deadlock-free-matrix.pdf';
    set_figure_properties(figHandle,'preset','paper')
    export_fig(figHandle, fullfile(folder_path,filename));
    close(figHandle);
end