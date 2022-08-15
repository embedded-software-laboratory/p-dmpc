%% Generate data: different allowed number of computation levels and different sample time
dt_s = 0.2;
max_num_CLs_all = [1:10,20];

e_CLs = cell(length(dt_s),length(max_num_CLs_all));

% prepare simulation options
options = OptionsMain;
options.consider_RSS = false;
options.is_sim_lab = true;
options.customResultName = '';
options.scenario = 'Commonroad';
options.trim_set = 9;
options.Hp = 5;
options.amount = 20;
options.T_end = 20;
options.priority = 'right_of_way_priority';
options.isPB = true;
options.isParl = true;
options.isAllowInheritROW = false;
options.strategy_consider_veh_without_ROW = '3';
options.isSaveResult = true;
options.visu = [false,false];
options.is_eval = false;
options.visualize_reachable_set = false;

for i_dt = 1:length(dt_s)
    for i_CL = 1:length(max_num_CLs_all)
        options.dt = dt_s(i_dt);
        options.max_num_CLs = max_num_CLs_all(i_CL);

        if options.max_num_CLs==options.amount
            % Sequential trajectory planning: Vehicles are allowed to enter
            % lanelet crossing areas
            options.strategy_enter_lanelet_crossing_area = '1';
        else
            options.strategy_enter_lanelet_crossing_area = '4';
        end

        full_path = FileNameConstructor.get_results_full_path(options);
        if isfile(full_path)
            disp('File already exists.')
        else
            % run simulation
            if exist('options','var') && exist('scenario','var')
                [~,~,~] = main(options,scenario);
            else
                [~,scenario,~] = main(options);
            end
            disp('Pause to cool the CPU...')
            pause(3) 
        end

        % data processing
        e_CLs{i_dt,i_CL} = EvaluationParl(options);

        disp([num2str(i_CL) ': done.'])
    end    
end

% get free flow speed, i.e., the speed that vehicles could travel if they are not influenced by others
% vehicles
if exist('Scenario','var')
    free_flow_speed = FreeFlowSpeed(scenario);
else
    free_flow_speed = FreeFlowSpeed();
end
disp('--------Finished--------')

%% Plot
set(0,'DefaultTextFontname', 'Times New Roman');
set(0,'DefaultAxesFontName', 'Times New Roman');
set(0,'defaultTextFontSize',9)
set(0,'defaultAxesFontSize',9)

for i_dt = 1:length(dt_s)
    dt = dt_s(i_dt);
    runtime_s = cellfun(@(c) c.runtime_total_per_step, e_CLs(i_dt,:), 'UniformOutput',false);
    speed_s = cellfun(@(c) c.average_speed_each_veh, e_CLs(i_dt,:), 'UniformOutput',false);
    fallback_rate = cellfun(@(c) c.fallback_rate*100, e_CLs(i_dt,:));
    grp_runtime = cell2mat(arrayfun(@(i){i*ones(numel(runtime_s{i}),1)},(1:numel(runtime_s))')); 
    grp_speed = cell2mat(arrayfun(@(i){i*ones(numel(speed_s{i}),1)},(1:numel(speed_s))')); 

    fig_x = 14;     fig_y = 6; % [cm]
    x_margin = 0;   y_margin = 0; 
    fig_x_position = fig_x - 2*x_margin;
    fig_y_position = fig_y - 2*y_margin;

    fig = figure('Name','differentAllowedCLs');
    set(fig, 'Units','centimeters', 'Position',[0 0 fig_x_position fig_y_position]/2)
    set(fig, 'PaperUnits','centimeters','PaperSize',[fig_x fig_y],'PaperOrientation','portrait',...
        'PaperPosition', [x_margin y_margin fig_x_position fig_y_position])

    t_fig = tiledlayout(1,3,'Padding','compact','TileSpacing','tight');

    % boxplot runtime
%     subplot(3,1,1)
    nexttile
    grid on
    hold on 
    boxplot(vertcat(runtime_s{:}),grp_runtime,'Colors','k','OutlierSize',4,'Labels',num2str(max_num_CLs_all'))
    % plot line indicating the sample time 
    y_line_dt = yline(dt,'--b','LineWidth',0.5);
%     p_dt = plot([0,length(runtime_s)+1],[dt,dt],'--b','LineWidth',0.5);
    xlabel({'$n_{CLs}$','(a) Computation time per step.'},'Interpreter','latex');
    ylabel('$t_c\:[s]$','Interpreter','latex')
    ylim([0,0.25])
    legend([y_line_dt],{'Sample time'},'FontSize',5,'Location','southeast','Interpreter','latex')

    % boxplot speed
%     subplot(3,1,2)
    nexttile
    grid on
    hold on
    boxplot(vertcat(speed_s{:}),grp_speed,'Colors','k','OutlierSize',4,'Labels',num2str(max_num_CLs_all'))
    free_flow_speed_i = free_flow_speed.free_flow_speed(free_flow_speed.sample_time==dt);
    y_line_FFS = yline(free_flow_speed_i,'--b','LineWidth',0.5);
    legend([y_line_FFS],{'Free flow speed'},'FontSize',5,'Location','southeast','Interpreter','latex')

    xlabel({'$n_{CLs}$','(b) Average speed of each vehicle.'},'Interpreter','latex')
    ylabel('$\overline{v}\:[m/s]$','Interpreter','latex')
    ylim([0.38,0.75])

    % plot fallback rate
%     subplot(3,1,3)
    nexttile
    grid on
    hold on
    plot(categorical(max_num_CLs_all),fallback_rate,'-*k','LineWidth',0.5,'MarkerSize',4)
%     bar(max_num_CLs_all,fallback_rate)
%     xlim([max_num_CLs_all(1)-1,max_num_CLs_all(end)+0.5])
    ylim([0,3])
    xlabel({'$n_{CLs}$','(c) Fallback rate.'},'Interpreter','latex')
    ylabel('$p_{FB}\:[\%]$','Interpreter','latex')

%     xticks(max_num_CLs_all)
    
%     xlabel(t_fig,'Allowed number of computation levels','FontSize',9,'FontName','Times New Roman')
%     title(t_fig,['Sample time: ' num2str(dt) ' seconds, number of vehicles: 20'],'FontSize',9,'FontName','Times New Roman')
    
    % save fig
    file_name = ['differentMaximumAllowedNumberOfCLs_dt' num2str(dt)];
    e_CLs{1}.save_fig(fig,file_name)
end
