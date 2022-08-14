%% Evaluate the vehicle prioritizing algorithm: maximum number of vehicles
priority_assign_options = {'right_of_way_priority','random_priority','constant_priority'};
e_differentNumVehs = cell(length(priority_assign_options),1);

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
options.max_num_CLs = 3;
options.strategy_consider_veh_without_ROW = '3';
options.strategy_enter_lanelet_crossing_area = '4';
options.isSaveResult = true;
options.visu = [false,false];
options.is_eval = false;
options.visualize_reachable_set = false;

for i_priority = 1:length(priority_assign_options)
    options.priority = priority_assign_options{i_priority};
    % After preexamining, maximum 17 (14) vehicles can run collision- and deadlock-free
    % if random (constant) priority assignment is used
    switch options.priority
        case 'right_of_way_priority'
            options.amount = 34;
        case 'random_priority'
            options.amount = 17;
        case 'constant_priority'
            options.amount = 14;
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
        disp('Pausing...')
        pause(3) % pause to cool the machine
    end

    % evaluate
    e_differentNumVehs{i_priority} = EvaluationParl(results_full_path);

    disp([num2str(i_priority) ': done.'])
end
disp('--------Finished--------')


%% Plot
set(0,'DefaultTextFontname', 'Times New Roman');
set(0,'DefaultAxesFontName', 'Times New Roman');
set(0,'DefaultTextFontSize',9)
set(0,'DefaultAxesFontSize',9)

nVeh_s = cellfun(@(c) c.nVeh, e_differentNumVehs);
speed_average_s = cellfun(@(c) c.average_speed, e_differentNumVehs);
speed_sum_s = cellfun(@(c) c.sum_speeds_all_vehicles, e_differentNumVehs);

computation_time_s = cellfun(@(c) c.runtime_total_per_step, e_differentNumVehs, 'UniformOutput',false);
grp_computation_time = cell2mat(arrayfun(@(i){i*ones(numel(computation_time_s{i}),1)},(1:numel(computation_time_s))')); 

fig_x = 5.5;     fig_y = 10; % [cm]
x_margin = 0;   y_margin = 0; 
fig_x_position = fig_x - 2*x_margin;
fig_y_position = fig_y - 2*y_margin;

fig = figure('Name','priorityAssignment');
set(fig, 'Units','centimeters', 'Position',[0 0 fig_x_position fig_y_position]/2)
set(fig, 'PaperUnits','centimeters','PaperSize',[fig_x fig_y],'PaperOrientation','portrait',...
    'PaperPosition', [x_margin y_margin fig_x_position fig_y_position])

t_fig = tiledlayout(3,1,'Padding','tight','TileSpacing','tight');
X_string = {'STAC-based','Random','Constant'};
X_cat = categorical(X_string);
X_cat = reordercats(X_cat,X_string);

nexttile
b1 = bar(X_cat,nVeh_s);
grid on
xtips1 = b1(1).XEndPoints;
ytips1 = b1(1).YEndPoints;
labels1 = string(b1(1).YData);
text(xtips1,ytips1,labels1,'HorizontalAlignment','center','VerticalAlignment','bottom')
xtickangle(0)
xlabel({'(a). Max. allowed number of vehicles.'})
ylabel('$n_{veh}^{max}$','Interpreter','latex')

nexttile
b2 = bar(X_cat,[speed_average_s,speed_sum_s]);
grid on
xtips1 = b2(1).XEndPoints;
ytips1 = b2(1).YEndPoints;
labels1 = string(round(b2(1).YData,2));
text(xtips1-0.03,ytips1,labels1,'HorizontalAlignment','center','VerticalAlignment','bottom')
xtips2 = b2(2).XEndPoints;
ytips2 = b2(2).YEndPoints;
labels2 = string(round(b2(2).YData,2));
text(xtips2,ytips2,labels2,'HorizontalAlignment','center','VerticalAlignment','bottom')
xlabel('(b). Average speed and sum of speeds.')
ylabel('$\overline{v}\:[m/s]$','Interpreter','latex')
legend({'Average','Sum'},'Location','best')
ylim([0 20])

nexttile
boxplot(vertcat(computation_time_s{:}),grp_computation_time,'Colors','k','OutlierSize',4,'Labels',X_string);
grid on
hold on
y_line_dt = yline(options.dt,'b--','LineWidth',.5);
ylim([0 0.4])
xlabel('(c). Computation time per step.')
ylabel('$t_c\:[s]$','Interpreter','latex')
legend(y_line_dt,'Sample time','Location','northeast')
% title(t_fig,'Allowed number of computation levels: 3','FontSize',9,'FontName','Times New Roman')
% xlabel(t_fig,{'Priority Assignment Strategies'},'FontSize',9,'FontName','Times New Roman')

% save fig
file_name = 'differentPriorityAssignmentStrategies_differentNumVehs';
e_differentNumVehs{1}.save_fig(fig,file_name)
%% Evaluate the vehicle prioritizing algorithm: use the same number of vehicle
priority_assign_options = {'right_of_way_priority','random_priority','constant_priority'};
e_sameNumVehs = cell(length(priority_assign_options),1);

% prepare simulation options
options = OptionsMain;
options.consider_RSS = false;
options.is_sim_lab = true;
options.customResultName = '';
options.scenario = 'Commonroad';
options.trim_set = 9;
options.Hp = 5;
options.dt = 0.2;
options.T_end = 30;
options.isPB = true;
options.isParl = true;
options.isAllowInheritROW = true;
options.max_num_CLs = 3;
options.strategy_consider_veh_without_ROW = '3';
options.strategy_enter_lanelet_crossing_area = '4';
options.isSaveResult = true;
options.visu = [false,false];
options.is_eval = false;
options.visualize_reachable_set = false;

for i_priority = 1:length(priority_assign_options)
    options.priority = priority_assign_options{i_priority};
    % After preexamining, maximum 17 (14) vehicles can run collision- and deadlock-free
    % if random (constant) priority assignment is used
    options.amount = 14;

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
        disp('Pausing...')
        pause(3) % pause to cool the machine
    end

    % evaluate
    e_sameNumVehs{i_priority} = EvaluationParl(results_full_path);

    disp([num2str(i_priority) ': done.'])
end
disp('--------Finished--------')


%% Plot
set(0,'DefaultTextFontname', 'Times New Roman');
set(0,'DefaultAxesFontName', 'Times New Roman');
set(0,'defaultTextFontSize',9)
set(0,'defaultAxesFontSize',9)

speed_average_s = cellfun(@(c) c.average_speed_each_veh, e_sameNumVehs, 'UniformOutput', false);
grp_speed = cell2mat(arrayfun(@(i){i*ones(numel(speed_average_s{i}),1)},(1:numel(speed_average_s))')); 

computation_time_s = cellfun(@(c) c.runtime_total_per_step, e_sameNumVehs, 'UniformOutput', false);
grp_computation_time = cell2mat(arrayfun(@(i){i*ones(numel(computation_time_s{i}),1)},(1:numel(computation_time_s))')); 


fallback_rate_s = cellfun(@(c) c.fallback_rate, e_sameNumVehs);

fig_x = 5.5;     fig_y = 8; % [cm]
x_margin = 0;   y_margin = 0; 
fig_x_position = fig_x - 2*x_margin;
fig_y_position = fig_y - 2*y_margin;

fig = figure('Name','priorityAssignment');
set(fig, 'Units','centimeters', 'Position',[0 0 fig_x_position fig_y_position]/2)
set(fig, 'PaperUnits','centimeters','PaperSize',[fig_x fig_y],'PaperOrientation','portrait',...
    'PaperPosition', [x_margin y_margin fig_x_position fig_y_position])

t_fig = tiledlayout(2,1,'Padding','none','TileSpacing','compact');
X_string = {'STAC-based','Random','Constant'};
X_cat = categorical(X_string);
X_cat = reordercats(X_cat,X_string);

nexttile
% average speed
b1 = bar(X_cat,nVeh_s);
boxplot(vertcat(speed_average_s{:}),grp_speed,'Colors','k','OutlierSize',4,'Labels',X_string);
grid on
xlabel('(a). Average speed of each vehicle.')
ylabel('$\overline{v}\:[m/s]$','Interpreter','latex')

nexttile
% computation time per step
boxplot(vertcat(computation_time_s{:}),grp_computation_time,'Colors','k','OutlierSize',4,'Labels',X_string);
grid on
hold on
y_line_dt = yline(options.dt,'b--','LineWidth',.5);
ylim([0 0.21])
xlabel('(b). Computation time per step.')
ylabel('$t_c\:[s]$','Interpreter','latex')
legend(y_line_dt,'Sample time','Location','southeast')
% title(t_fig,'Allowed number of computation levels: 3','FontSize',9,'FontName','Times New Roman')
% xlabel(t_fig,{'Priority Assignment Strategies'},'FontSize',9,'FontName','Times New Roman')


% save fig
file_name = 'differentPriorityAssignmentStrategies_sameNumVehs';
e_differentNumVehs{1}.save_fig(fig,file_name)
