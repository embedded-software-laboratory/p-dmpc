%% Generate data: strategieis to improve feasibilities
    % 1. Higher-priority vehicles consider emergency braking maneuver of
    % lowr-priority vehicles
    % 2. 
strategy_feasibility_deadlock = {{'3','4'},{'3','1'},{'1','4'},{'1','1'}};
nVeh_s = 30:2:40;

% prepare simulation options
options = OptionsMain;
options.consider_RSS = false;
options.is_sim_lab = true;
options.customResultName = '';
options.scenario_name = 'Commonroad';
options.trim_set = 9;
options.Hp = 5;

options.T_end = 300;
options.dt = 0.2;
options.max_num_CLs = 4;
options.priority = 'STAC_priority';
options.isPB = true;
options.isParl = true;
options.isAllowInheritROW = false;
options.isSaveResult = true;
options.visu = [false,false];
options.is_eval = false;
options.visualize_reachable_set = false;

random_times = 3;
e_feasibility_differentNumVehs = cell(length(strategy_feasibility_deadlock)*random_times,length(nVeh_s));
n_simulations = numel(e_feasibility_differentNumVehs);
count = 0;

for i = 1:length(strategy_feasibility_deadlock)
    for iVeh = 1:length(nVeh_s)
        options.strategy_consider_veh_without_ROW = strategy_feasibility_deadlock{i}{1};
        options.strategy_enter_lanelet_crossing_area = strategy_feasibility_deadlock{i}{2};
        options.amount = nVeh_s(iVeh);

        random_seed = RandStream('mt19937ar');
        for iRandom=1:random_times

            options.random_idx =  iRandom;
            options.veh_ids = sort(randsample(random_seed,1:40,options.amount),'ascend');
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
                pause(5)
            end

            % data processing
            e_feasibility_differentNumVehs{(i-1)*random_times+iRandom,iVeh} = EvaluationParl(options);
        
            % display progress
            count = count + 1;
            disp(['--------Progress ' num2str(count) '/' num2str(n_simulations) ': done--------'])
        end
    end
end
disp('--------Finished--------')
%% test
options.visu(1) = 1;
options.isSaveResult = false;
options.amount = 40;
options.random_idx =  iRandom;
options.veh_ids = sort(randsample(random_seed,1:40,options.amount),'ascend');
options.strategy_consider_veh_without_ROW = '1';
options.strategy_enter_lanelet_crossing_area = '1';
[~,~,~] = main(options,scenario);

%% Plot
set(0,'DefaultTextFontname', 'Times New Roman');
set(0,'DefaultAxesFontName', 'Times New Roman');
set(0,'defaultTextFontSize',11)
set(0,'defaultAxesFontSize',11)

average_random_simulations = @(data) reshape(mean(reshape(data,random_times,[]),1),length(strategy_feasibility_deadlock),[]);

% total runtime
t_total_tmp = cellfun(@(c) c.t_total, e_feasibility_differentNumVehs);
t_total = average_random_simulations(t_total_tmp);
% speed_average_s = cellfun(@(c) c.average_speed, e_feasibility_differentNumVehs);
fallback_rate_tmp = cellfun(@(c) c.fallback_rate*100, e_feasibility_differentNumVehs);
fallback_rate = average_random_simulations(fallback_rate_tmp);
% speed_sum_s = cellfun(@(c) c.sum_speeds_all_vehicles, e_feasibility_differentNumVehs);
is_deadlock_tmp = cellfun(@(c) c.is_deadlock,e_feasibility_differentNumVehs);
deadlock_rate = average_random_simulations(is_deadlock_tmp);
collision_rate = (1-deadlock_rate)*100;

clear plot_line_options
plot_line_options(1) = struct('LineWidth',0.6,'Color','#A2142F','LineStyle','-','Marker','*','MarkerSize',4);
plot_line_options(2) = struct('LineWidth',0.6,'Color','#7E2F8E','LineStyle','-','Marker','^','MarkerSize',4);
plot_line_options(3) = struct('LineWidth',0.6,'Color','#0072BD','LineStyle','-','Marker','o','MarkerSize',4);
plot_line_options(4) = struct('LineWidth',0.6,'Color','#D95319','LineStyle','-','Marker','v','MarkerSize',4);
plot_line_options(5) = struct('LineWidth',0.6,'Color','#EDB120','LineStyle','-','Marker','s','MarkerSize',4);
plot_line_options(6) = struct('LineWidth',0.6,'Color','#77AC30','LineStyle','-','Marker','d','MarkerSize',4);

legend_ = {'Use both strategies','Feasibility improving strategy','Vehicle decoupling strategy','None'};
fig_x = 12;     fig_y = 6; % [cm]
x_margin = 0;   y_margin = 0; 
fig_x_position = fig_x - 2*x_margin;
fig_y_position = fig_y - 2*y_margin;

file_name = 'evalFeasibilityDeadlock_differentNumVehs';
fig = figure('Name',file_name);
set(fig, 'Units','centimeters', 'Position',[0 0 fig_x_position fig_y_position]/2)
set(fig, 'PaperUnits','centimeters','PaperSize',[fig_x fig_y],'PaperOrientation','portrait',...
    'PaperPosition', [0 -0.15 fig_x_position fig_y_position+0.2])

t_fig = tiledlayout(1,2,'Padding','compact','TileSpacing','compact');

% maximum runtime
nexttile
grid on
hold on
p_t_total(1) = plot(nVeh_s,t_total(1,:),plot_line_options(1));
p_t_total(2) = plot(nVeh_s,t_total(2,:),plot_line_options(2));
p_t_total(3) = plot(nVeh_s,t_total(3,:),plot_line_options(3));
p_t_total(4) = plot(nVeh_s,t_total(4,:),plot_line_options(4));
ylim([0 150])
xlabel({'$n_{veh}$','(a) Maximum runtime.'},'Interpreter','latex')
ylabel('$t_{max}\:[s]$','Interpreter','latex')
xticks(nVeh_s)
t_total_sum = sum(t_total,2);
disp(['Using both strategies: the maximum runtime is ' num2str(t_total_sum(1)/t_total_sum(4)) ' times of that of not using any of them.'])
disp(['Using both strategies: the maximum runtime is ' num2str(t_total_sum(2)/t_total_sum(4)) ' times of that of not using any of them.'])
disp(['Using both strategies: the maximum runtime is ' num2str(t_total_sum(3)/t_total_sum(4)) ' times of that of not using any of them.'])
% legend(p_t_total,legend_,'Location',)

% % fallback rate
% nexttile(2,[1,1])
% grid on
% hold on
% p_fallback_rate(1) = plot(nVeh_s,fallback_rate(1,:),plot_line_options(1));
% p_fallback_rate(2) = plot(nVeh_s,fallback_rate(2,:),plot_line_options(2));
% p_fallback_rate(3) = plot(nVeh_s,fallback_rate(3,:),plot_line_options(3));
% p_fallback_rate(4) = plot(nVeh_s,fallback_rate(4,:),plot_line_options(4));
% ylim([0,4])
% xticks(nVeh_s)
% xlabel({'(b) Average fallback rate.'},'Interpreter','latex')
% ylabel('$\overline{p}_{FB}\:[\%]$','Interpreter','latex')

% global legend
lg  = legend(p_t_total,legend_,'Orientation','Horizontal','NumColumns',2); 
lg.Layout.Tile = 'North';

% is deadlock
nexttile
grid on 
hold on
% box on
p_collision_rate(1) = plot(nVeh_s,collision_rate(1,:),plot_line_options(1));
p_collision_rate(2) = plot(nVeh_s,collision_rate(2,:),plot_line_options(2));
p_collision_rate(3) = plot(nVeh_s,collision_rate(3,:),plot_line_options(3));
p_collision_rate(4) = plot(nVeh_s,collision_rate(4,:),plot_line_options(4));
ylim([0 100])
xticks(nVeh_s)
yticks(round((0:random_times)/random_times*100,1))
xlabel({'$n_{veh}$','(b) Collision possibility.'},'Interpreter','latex')
ylabel('$p_{co}\:[\%]$','Interpreter','latex')

% save fig
e_feasibility_differentNumVehs{1}.save_fig(fig,file_name)
%% Generate data: strategieis to improve feasibilities (same number of vehicles)
    % 1. Higher-priority vehicles consider emergency braking maneuver of
    % lowr-priority vehicles
    % 2. 
strategy_feasibility_deadlock = {{'3','4'},{'3','1'},{'1','4'},{'1','1'}};


% prepare simulation options
options = OptionsMain;
options.consider_RSS = false;
options.is_sim_lab = true;
options.customResultName = '';
options.scenario_name = 'Commonroad';
options.trim_set = 9;
options.Hp = 5;

options.T_end = 20;
options.dt = 0.2;
options.amount = 20;
options.max_num_CLs = options.amount;
options.priority = 'STAC_priority';
options.isPB = true;
options.isParl = true;
options.isAllowInheritROW = false;
options.isSaveResult = true;
options.visu = [false,false];
options.is_eval = false;
options.visualize_reachable_set = false;

random_times = 1;
e_feasibility_sameNumVehs = cell(random_times,length(strategy_feasibility_deadlock));
n_simulations = numel(e_feasibility_sameNumVehs);
count = 0;
for i = 1:length(strategy_feasibility_deadlock)
    options.strategy_consider_veh_without_ROW = strategy_feasibility_deadlock{i}{1};
    options.strategy_enter_lanelet_crossing_area = strategy_feasibility_deadlock{i}{2};
    
    random_seed = RandStream('mt19937ar');
    for iRandom=1:random_times
        options.random_idx =  iRandom;
        options.veh_ids = sort(randsample(random_seed,1:40,options.amount),'ascend');
    
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
            pause(5)
        end

        % data processing
        e_feasibility_sameNumVehs{iRandom,i} = EvaluationParl(options);
    
        % display progress
        count = count + 1;
        disp(['--------Progress ' num2str(count) '/' num2str(n_simulations) ': done--------'])
    end
end
disp('--------Finished--------')

%% Plot
set(0,'DefaultTextFontname', 'Times New Roman');
set(0,'DefaultAxesFontName', 'Times New Roman');
set(0,'defaultTextFontSize',11)
set(0,'defaultAxesFontSize',11)

average_random_simulations = @(data) reshape(mean(reshape(data,random_times,[]),1),length(strategy_feasibility_deadlock),[]);

% total runtime
% t_total_tmp = cellfun(@(c) c.t_total, e_feasibility_differentNumVehs);
% t_total = average_random_simulations(t_total_tmp);
fallback_rate_tmp = cellfun(@(c) c.fallback_rate*100, e_feasibility_sameNumVehs);
fallback_rate = average_random_simulations(fallback_rate_tmp);
% speed_sum_s = cellfun(@(c) c.sum_speeds_all_vehicles, e_feasibility_differentNumVehs);

speed_average_tmp = cellfun(@(c) c.average_speed, e_feasibility_sameNumVehs);
speed_average = average_random_simulations(speed_average_tmp);

speed_average_each_veh_tmp = cellfun(@(c) c.average_speed_each_veh, e_feasibility_sameNumVehs, 'UniformOutput',false);
speed_average_each_veh = cell(1,length(strategy_feasibility_deadlock));
for i = 1:size(speed_average_each_veh_tmp,2)
    speed_average_each_veh{i} = vertcat(speed_average_each_veh_tmp{:,i});
end
grp_speed_average_each_veh = cell2mat(arrayfun(@(i){i*ones(numel(speed_average_each_veh{i}),1)},(1:numel(speed_average_each_veh))')); 

CT_graph_search_tmp = cellfun(@(c) c.runtime_graph_search, e_feasibility_sameNumVehs);
CT_graph_search = average_random_simulations(CT_graph_search_tmp);
CT_total_max_tmp = cellfun(@(c) c.runtime_max, e_feasibility_sameNumVehs);
CT_total_max = average_random_simulations(CT_total_max_tmp);
CT_total_average_tmp = cellfun(@(c) c.runtime_total_average, e_feasibility_sameNumVehs);
CT_total_average = average_random_simulations(CT_total_average_tmp);
X_string = {'Both','FIS','DAS','None'};
X_cat = categorical(X_string);
X_cat = reordercats(X_cat,X_string);

fig_x = 12;     fig_y = 7.1; % [cm]
x_margin = 0;   y_margin = 0; 
fig_x_position = fig_x - 2*x_margin;
fig_y_position = fig_y - 2*y_margin;

file_name = 'evalFeasibilityDeadlock_sameNumVehs';
fig = figure('Name',file_name);
set(fig, 'Units','centimeters', 'Position',[0 0 fig_x_position fig_y_position]/2)
set(fig, 'PaperUnits','centimeters','PaperSize',[fig_x fig_y],'PaperOrientation','portrait',...
    'PaperPosition', [x_margin 0.03 fig_x_position-0.1 fig_y_position-0.01])

t_fig = tiledlayout(5,2,'Padding','tight','TileSpacing','compact');

% average speed
nexttile(1,[3,1])
b1 = boxplot(vertcat(speed_average_each_veh{:}),grp_speed_average_each_veh,'Colors','k','OutlierSize',4,'Labels',X_string);
hAx=gca;                                
xtk=hAx.XTick;
grid on
hold on
% p = plot(xtk,horzcat(speed_average_s{:}));
s = scatter(xtk,speed_average,12,'filled','o','MarkerFaceColor',[0 0.4470 0.7410]);
set(gca, 'XTickLabel','')
legend(s,'Average','Location','south')
ylim([0.5,0.8])
xlabel('(a) Speed of each vehicle.')
ylabel('$\overline{v}\:[m/s]$','Interpreter','latex')
xtickangle(0)

nexttile(7,[2,1])
% fallback rate
plot(X_cat,fallback_rate,'LineWidth',0.6,'Color','#0072BD','LineStyle','-','Marker','o','MarkerSize',4)
xlabel('(b) Average fallback rate.')
ylabel('$\overline{p}_{FB}\:[\%]$','Interpreter','latex')
grid on
ylim([0 3])
% xlabel(t_fig,'Improvement of Feasibility','FontSize',9,'FontName','Times New Roman')
xtickangle(0)

% plot computation time
nexttile(2,[5,1])
b2 = bar(X_cat,[CT_total_average,CT_total_max,CT_graph_search]);
% set(gca, 'XTickLabel','')
grid on
legend({'Total (avg.)','Total (max.)','Plan Trajectory (max.)'},'Location','best','FontSize',9,'Orientation','horizontal','NumColumns',1)
xlabel('(c) Computation time per step.')
ylabel('$t_c\:[s]$','Interpreter','latex')
ylim([0,0.5])
xtickangle(0)

% save fig
e_feasibility_sameNumVehs{1}.save_fig(fig,file_name)


