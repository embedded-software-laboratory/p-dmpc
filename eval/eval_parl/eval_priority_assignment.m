%% Evaluate the vehicle prioritizing algorithm: maximum number of vehicles
priority_assign_options = {'right_of_way_priority','random_priority','constant_priority'};

% prepare simulation options
options = OptionsMain;
options.consider_RSS = false;
options.is_sim_lab = true;
options.customResultName = '';
options.scenario = 'Commonroad';
options.trim_set = 9;
options.Hp = 5;
options.dt = 0.2;
options.T_end = 20;
options.isPB = true;
options.isParl = true;
options.isAllowInheritROW = true;
options.max_num_CLs = 4;
options.strategy_consider_veh_without_ROW = '3';
options.strategy_enter_lanelet_crossing_area = '4';
options.isSaveResult = true;
options.visu = [false,false];
options.is_eval = false;
options.visualize_reachable_set = false;

e_differentNumVehs = cell(length(priority_assign_options),1);
n_simulations = numel(e_differentNumVehs);
count = 0;

for i_priority = 1:length(priority_assign_options)
    options.priority = priority_assign_options{i_priority};
    % After preexamining, maximum 15 (18) vehicles can run collision- and deadlock-free
    % if random (constant) priority assignment is used
    switch options.priority
        case 'right_of_way_priority'
            options.amount = 40;
        case 'random_priority'
            options.amount = 15;
        case 'constant_priority'
            options.amount = 18;
    end

    results_full_path = FileNameConstructor.get_results_full_path(options);
    if isfile(results_full_path)
        disp('File already exists.')
    else
        % run simulation
        if exist('options','var') && exist('scenario','var')
            [~,~,~] = main(options,scenario);
        else
            [~,scenario,~] = main(options);
        end
        pause(10) % pause to cool the machine
    end

    % evaluate
    e_differentNumVehs{i_priority} = EvaluationParl(results_full_path);
    
    % display progress
    count = count + 1;
    disp(['--------Progress ' num2str(count) '/' num2str(n_simulations) ': done--------'])
end
disp('--------Finished--------')


%% Plot
set(0,'DefaultTextFontname', 'Times New Roman');
set(0,'DefaultAxesFontName', 'Times New Roman');
set(0,'defaultTextFontSize',11)
set(0,'defaultAxesFontSize',11)

nVeh_s = cellfun(@(c) c.nVeh, e_differentNumVehs);
speed_average_s = cellfun(@(c) c.average_speed, e_differentNumVehs);
speed_sum_s = cellfun(@(c) c.sum_speeds_all_vehicles, e_differentNumVehs);

computation_time_s = cellfun(@(c) c.runtime_max, e_differentNumVehs, 'UniformOutput',false);
grp_computation_time = cell2mat(arrayfun(@(i){i*ones(numel(computation_time_s{i}),1)},(1:numel(computation_time_s))')); 



% computation time of different parts
CT_group_vehs = cellfun(@(c) c.runtime_group_vehs, e_differentNumVehs);
CT_determine_couplings = cellfun(@(c) c.runtime_determine_couplings, e_differentNumVehs);

CT_assign_priority = cellfun(@(c) c.runtime_assign_priority, e_differentNumVehs);

CT_graph_search = cellfun(@(c) c.runtime_graph_search, e_differentNumVehs);


CT_total_each_step = cellfun(@(c) c.runtime_total_per_step, e_differentNumVehs, 'UniformOutput',false);
grp_CT_total_each_step = cell2mat(arrayfun(@(i){i*ones(numel(CT_total_each_step{i}),1)},(1:numel(CT_total_each_step))')); 

CT_total_average = cellfun(@mean, CT_total_each_step);

CT_total_max = cellfun(@(c) c.runtime_max, e_differentNumVehs);

% total runtime
t_total = cellfun(@(c) c.t_total, e_differentNumVehs);

% number of coupling 
num_couplings = cellfun(@(c) c.num_couplings, e_differentNumVehs);
num_couplings_ignored = cellfun(@(c) c.num_couplings_ignored, e_differentNumVehs);
num_couplings_between_grps = cellfun(@(c) c.num_couplings_between_grps, e_differentNumVehs);
num_couplings_between_grps_ignored = cellfun(@(c) c.num_couplings_between_grps_ignored, e_differentNumVehs);

% maximum numbero of actual computation levels
CLs_num_max = cellfun(@(c) c.CLs_num_max, e_differentNumVehs);
% fallback rate
fallback_rate = cellfun(@(c) c.fallback_rate*100, e_differentNumVehs);

% average speed
speed_average = cellfun(@(c) c.average_speed, e_differentNumVehs);

speed_each_veh = cellfun(@(c) c.average_speed_each_veh, e_differentNumVehs, 'UniformOutput',false);
grp_speed_each_veh = cell2mat(arrayfun(@(i){i*ones(numel(speed_each_veh{i}),1)},(1:numel(speed_each_veh))')); 

clear plot_line_options
plot_line_options(1) = struct('LineWidth',0.5,'Color','#A2142F','LineStyle','-','Marker','*','MarkerSize',4);
plot_line_options(2) = struct('LineWidth',0.5,'Color','#7E2F8E','LineStyle','-','Marker','^','MarkerSize',4);
plot_line_options(3) = struct('LineWidth',0.5,'Color','#0072BD','LineStyle','-','Marker','o','MarkerSize',4);
plot_line_options(4) = struct('LineWidth',0.5,'Color','#D95319','LineStyle','-','Marker','v','MarkerSize',4);
plot_line_options(5) = struct('LineWidth',0.5,'Color','#EDB120','LineStyle','-','Marker','s','MarkerSize',4);
plot_line_options(6) = struct('LineWidth',0.5,'Color','#77AC30','LineStyle','-','Marker','d','MarkerSize',4);



fig_x = 8;     fig_y = 12; % [cm]
x_margin = 0;   y_margin = 0; 
fig_x_position = fig_x - 2*x_margin;
fig_y_position = fig_y - 2*y_margin;

file_name = 'evalPriorityAssignment_differentNumVehs';
fig = figure('Name',file_name);
set(fig, 'Units','centimeters', 'Position',[0 0 fig_x_position fig_y_position]/2)
set(fig, 'PaperUnits','centimeters','PaperSize',[fig_x fig_y],'PaperOrientation','portrait',...
    'PaperPosition', [x_margin y_margin fig_x_position fig_y_position])

t_fig = tiledlayout(2,1,'Padding','compact','TileSpacing','compact');
X_string = {'STAC-based','Random','Constant'};
X_cat = categorical(X_string);
X_cat = reordercats(X_cat,X_string);

% maximum controllable vehicles
nexttile
b1 = bar(X_cat,nVeh_s);
grid on
xtips1 = b1(1).XEndPoints;
ytips1 = b1(1).YEndPoints;
labels1 = string(b1(1).YData);
text(xtips1,ytips1,labels1,'HorizontalAlignment','center','VerticalAlignment','bottom')
xlabel({'(a). Maximum controllable vehicles.'})
ylabel('$n_{veh}^{max}$','Interpreter','latex')
ylim([0 48])
xtickangle(0)

% average speed
nexttile
b2 = bar(X_cat,[speed_average_s,speed_sum_s]);
grid on
xtips1 = b2(1).XEndPoints;
ytips1 = b2(1).YEndPoints;
labels1 = string(round(b2(1).YData,2));
text(xtips1,ytips1,labels1,'HorizontalAlignment','center','VerticalAlignment','bottom')
xtips2 = b2(2).XEndPoints;
ytips2 = b2(2).YEndPoints;
labels2 = string(round(b2(2).YData,2));
text(xtips2,ytips2,labels2,'HorizontalAlignment','center','VerticalAlignment','bottom')
xlabel('(b). Average speed and sum of speeds.')
ylabel('$\overline{v},v_{sum}\:[m/s]$','Interpreter','latex')
legend({'Average','Sum'},'Location','northeast')
ylim([0 25])
xtickangle(0)

% % plot computation time
% nexttile
% b3 = bar(X_cat,[CT_total_average,CT_total_max]);
% grid on
% xtips1 = b3(1).XEndPoints;
% ytips1 = b3(1).YEndPoints;
% labels1 = string(round(b3(1).YData,2));
% text(xtips1-0.03,ytips1,labels1,'HorizontalAlignment','center','VerticalAlignment','bottom')
% xtips2 = b3(2).XEndPoints;
% ytips2 = b3(2).YEndPoints;
% labels2 = string(round(b3(2).YData,2));
% text(xtips2,ytips2,labels2,'HorizontalAlignment','center','VerticalAlignment','bottom')
% legend({'Average','Maximum'},'Location','northeast')
% xlabel({'(c) Total computation time per step.'},'Interpreter','latex');
% ylabel('$t_c\:[s]$','Interpreter','latex')
% ylim([0,0.42])
% xtickangle(0)
% title(t_fig,'Allowed number of computation levels: 3','FontSize',9,'FontName','Times New Roman')
% xlabel(t_fig,{'Priority Assignment Strategies'},'FontSize',9,'FontName','Times New Roman')

% save fig
e_differentNumVehs{1}.save_fig(fig,file_name)
%% Evaluate the vehicle prioritizing algorithm: use the same number of vehicle
priority_assign_options = {'right_of_way_priority','random_priority','constant_priority'};

% prepare simulation options
options = OptionsMain;
options.consider_RSS = false;
options.is_sim_lab = true;
options.customResultName = '';
options.scenario = 'Commonroad';
options.trim_set = 9;
options.Hp = 5;
options.dt = 0.2;
options.T_end = 20;
options.isPB = true;
options.isParl = true;
options.isAllowInheritROW = true;
options.max_num_CLs = 4;
options.strategy_consider_veh_without_ROW = '3';
options.strategy_enter_lanelet_crossing_area = '4';
options.isSaveResult = true;
options.visu = [false,false];
options.is_eval = false;
options.visualize_reachable_set = false;

n_simulations = numel(e_differentNumVehs);
count = 0;
e_sameNumVehs = cell(length(priority_assign_options),1);

for i_priority = 1:length(priority_assign_options)
    options.priority = priority_assign_options{i_priority};
    % Use the maximum number of vehicles that are controllable by all the three
    % priority assignment strategies
    options.amount = 15;

    results_full_path = FileNameConstructor.get_results_full_path(options);
    if isfile(results_full_path)
        disp('File already exists.')
    else
        % run simulation
        if exist('options','var') && exist('scenario','var')
            [~,~,~] = main(options,scenario);
        else
            [~,scenario,~] = main(options);
        end
        pause(10) % pause to cool the machine
    end

    % evaluate
    e_sameNumVehs{i_priority} = EvaluationParl(results_full_path);
    % display progress
    count = count + 1;
    disp(['--------Progress ' num2str(count) '/' num2str(n_simulations) ': done--------'])
end
disp('--------Finished--------')


%% Plot
set(0,'DefaultTextFontname', 'Times New Roman');
set(0,'DefaultAxesFontName', 'Times New Roman');
set(0,'defaultTextFontSize',11)
set(0,'defaultAxesFontSize',11)

speed_average_s = cellfun(@(c) c.average_speed_each_veh, e_sameNumVehs, 'UniformOutput', false);
grp_speed = cell2mat(arrayfun(@(i){i*ones(numel(speed_average_s{i}),1)},(1:numel(speed_average_s))')); 
speed_average = cellfun(@mean, speed_average_s);

computation_time_s = cellfun(@(c) c.runtime_total_per_step, e_sameNumVehs, 'UniformOutput', false);
grp_computation_time = cell2mat(arrayfun(@(i){i*ones(numel(computation_time_s{i}),1)},(1:numel(computation_time_s))')); 

fallback_rate_s = cellfun(@(c) c.fallback_rate, e_sameNumVehs);

fig_x = 8;     fig_y = 8; % [cm]
x_margin = 0;   y_margin = 0; 
fig_x_position = fig_x - 2*x_margin;
fig_y_position = fig_y - 2*y_margin;

file_name = 'evalPriorityAssignment_sameNumVehs';
fig = figure('Name',file_name);
set(fig, 'Units','centimeters', 'Position',[0 0 fig_x_position fig_y_position]/2)
set(fig, 'PaperUnits','centimeters','PaperSize',[fig_x fig_y],'PaperOrientation','portrait',...
    'PaperPosition', [x_margin y_margin fig_x_position fig_y_position])

t_fig = tiledlayout(1,1,'Padding','compact','TileSpacing','compact');
X_string = {'STAC-based','Random','Constant'};
X_cat = categorical(X_string);
X_cat = reordercats(X_cat,X_string);

nexttile
% average speed
b1 = boxplot(vertcat(speed_average_s{:}),grp_speed,'Colors','k','OutlierSize',4,'Labels',X_string);
hAx=gca;                                
xtk=hAx.XTick;  
grid on
hold on
% p = plot(xtk,horzcat(speed_average_s{:}));
s = scatter(xtk,speed_average,12,'filled','o','MarkerFaceColor',[0 0.4470 0.7410]);

legend(s,'Average','Location','southwest')
% xlabel('Speed of each vehicle.')
ylabel('$\overline{v}\:[m/s]$','Interpreter','latex')
xtickangle(0)


% title(t_fig,'Allowed number of computation levels: 3','FontSize',9,'FontName','Times New Roman')
% xlabel(t_fig,{'Priority Assignment Strategies'},'FontSize',9,'FontName','Times New Roman')


% save fig
e_differentNumVehs{1}.save_fig(fig,file_name)
